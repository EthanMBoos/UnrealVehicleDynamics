#include "UvdVehicleComponent.h"

#include <numbers>

#include "Cesium3DTileset.h"
#include "CesiumGeoreference.h"
#include "Chaos/ChaosEngineInterface.h"
#include "Common/UdpSocketBuilder.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Eigen/Eigenvalues"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "GameFramework/WorldSettings.h"
#include "HAL/CriticalSection.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformFile.h"
#include "HAL/PlatformFileManager.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformTime.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/ScopeLock.h"
#include "Physics/Experimental/PhysInterface_Chaos.h"
#include "Physics/Experimental/PhysicsThreadLibrary.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "SocketSubsystem.h"
#include "Sockets.h"
#include "UObject/SoftObjectPath.h"
#include "uvd/core.hpp"
#include "uvd/marine.hpp"

DEFINE_LOG_CATEGORY_STATIC(LogUvdVehicle, Log, All);

enum class EUvdControlFunction : uint8 {
  Aileron,
  Elevator,
  Rudder,
  Throttle,
};

enum class EUvdPlantType : uint8 {
  Aerosonde,
  FossenOtter,
};

struct FUvdPwmMapping {
  int32 ChannelIndex = 0;
  EUvdControlFunction Function = EUvdControlFunction::Aileron;
  uvd::PwmCalibration Calibration;
};

struct FUvdVisualConfiguration {
  FString AssetPath;
  double UniformScale = 1.0;
  FVector OffsetBodyM = FVector::ZeroVector;
  FRotator Rotation = FRotator::ZeroRotator;
};

struct FUvdVehicleRuntime {
  EUvdPlantType PlantType = EUvdPlantType::Aerosonde;
  uvd::AerosondeParameters AircraftParameters;
  uvd::FossenOtterModel OtterModel;
  uvd::OtterState OtterState;
  uvd::OtterEnvironment OtterEnvironment;
  uvd::OtterCommand OtterOpenLoopCommand;
  uvd::OtterCommand OtterControllerCommand;
  TArray<FUvdPwmMapping> PwmMappings;
  uvd::RigidBodyState InitialState;
  uvd::RigidBodyState PreviousState;
  uvd::AircraftCommand TrimCommand;
  uvd::AircraftCommand ControllerCommand;
  uvd::Vector3 WindNedMps = uvd::Vector3::Zero();
  uvd::GeodeticPosition Origin;
  FUvdVisualConfiguration Visual;
  double OriginAltitudeMslM = 0.0;
  double FixedDtS = 1.0 / 120.0;
  double DurationS = 0.0;
  int64 TilesetAssetId = 1;
  uint16 ControllerPort = 9002;
  uint16 ControlPort = 9003;
  FSocket* ControllerSocket = nullptr;
  FSocket* ControlSocket = nullptr;
  TSharedPtr<FInternetAddr> ControllerEndpoint;
  TArray<uint8> CachedReply;
  uint32 LastFrame = 0;
  uint16 ControllerFrameRateHz = 0;
  uint16 ControllerPwm[4] = {0, 0, 0, 0};
  uint64 Step = 0;
  uint64 PhysicsStep = 0;
  uint64 ReleaseStep = 0;
  double PhysicsTimeS = 0.0;
  uint64 MissedControllerFrames = 0;
  uint64 DuplicateControllerFrames = 0;
  bool HasFrame = false;
  bool HasReply = false;
  bool Released = false;
  bool ReleaseRequested = false;
  bool ReleaseBarrierRegistered = false;
  int32 ReleaseParticipants = 1;
  bool Failed = false;
  bool IgnoreDuplicateFramesUntilAdvance = false;
  bool OpenLoop = false;
  TUniquePtr<IFileHandle> TruthFile;
  TUniquePtr<IFileHandle> ExchangeFile;
  ACesiumGeoreference* Georeference = nullptr;
};

namespace {

struct FUvdReleaseBarrier {
  FCriticalSection Mutex;
  int32 Registered = 0;
  int32 Requested = 0;
  double TargetSimTimeS = -1.0;
};

FUvdReleaseBarrier ReleaseBarrier;

void RegisterReleaseBarrier(FUvdVehicleRuntime& Runtime) {
  if (Runtime.ReleaseParticipants <= 1) {
    return;
  }
  FScopeLock Lock(&ReleaseBarrier.Mutex);
  if (ReleaseBarrier.Registered == 0) {
    ReleaseBarrier.Requested = 0;
    ReleaseBarrier.TargetSimTimeS = -1.0;
  }
  ++ReleaseBarrier.Registered;
  Runtime.ReleaseBarrierRegistered = true;
}

void UnregisterReleaseBarrier(FUvdVehicleRuntime& Runtime) {
  if (!Runtime.ReleaseBarrierRegistered) {
    return;
  }
  FScopeLock Lock(&ReleaseBarrier.Mutex);
  --ReleaseBarrier.Registered;
  if (ReleaseBarrier.Registered == 0) {
    ReleaseBarrier.Requested = 0;
    ReleaseBarrier.TargetSimTimeS = -1.0;
  }
  Runtime.ReleaseBarrierRegistered = false;
}

const uvd::Matrix3 NedToUnreal =
    (uvd::Matrix3() << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0)
        .finished();
const uvd::Matrix3 FrdToUnrealBody =
    (uvd::Matrix3() << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0).finished();

uvd::Vector3 ToUvd(const FVector& Value) { return {Value.X, Value.Y, Value.Z}; }

FVector ToUnreal(const uvd::Vector3& Value) {
  return {Value.x(), Value.y(), Value.z()};
}

uvd::Matrix3 RotationMatrix(const FTransform& Transform) {
  const FVector X = Transform.TransformVectorNoScale(FVector::XAxisVector);
  const FVector Y = Transform.TransformVectorNoScale(FVector::YAxisVector);
  const FVector Z = Transform.TransformVectorNoScale(FVector::ZAxisVector);
  uvd::Matrix3 Rotation;
  Rotation << X.X, Y.X, Z.X, X.Y, Y.Y, Z.Y, X.Z, Y.Z, Z.Z;
  return Rotation;
}

FQuat ToUnrealQuaternion(const uvd::Matrix3& Rotation) {
  const FVector X{Rotation(0, 0), Rotation(1, 0), Rotation(2, 0)};
  const FVector Y{Rotation(0, 1), Rotation(1, 1), Rotation(2, 1)};
  const FVector Z{Rotation(0, 2), Rotation(1, 2), Rotation(2, 2)};
  FMatrix Matrix = FMatrix::Identity;
  Matrix.SetAxes(&X, &Y, &Z);
  return FQuat{Matrix};
}

uvd::Matrix3 BodyToNedRotationToUnreal(const uvd::Matrix3& BodyToNed) {
  return NedToUnreal * BodyToNed * FrdToUnrealBody;
}

uvd::Vector3 BodyForceToUnreal(const uvd::Vector3& ForceBodyN) {
  return 100.0 * FrdToUnrealBody * ForceBodyN;
}

uvd::Vector3 BodyMomentToUnreal(const uvd::Vector3& MomentBodyNm) {
  return -10000.0 * FrdToUnrealBody * MomentBodyNm;
}

uvd::Matrix3 BodyInertiaToUnreal(const uvd::Matrix3& InertiaBodyKgm2) {
  const uvd::Matrix3 AxialMap = -FrdToUnrealBody;
  return 10000.0 * AxialMap * InertiaBodyKgm2 * AxialMap.transpose();
}

double Number(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name) {
  return Object->GetNumberField(Name);
}

uvd::Vector3 JsonVector(const TSharedPtr<FJsonObject>& Object,
                        const TCHAR* Name) {
  const TArray<TSharedPtr<FJsonValue>>& Values = Object->GetArrayField(Name);
  return {Values[0]->AsNumber(), Values[1]->AsNumber(), Values[2]->AsNumber()};
}

bool JsonFVector(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name,
                 FVector& Result) {
  const TArray<TSharedPtr<FJsonValue>>& Values = Object->GetArrayField(Name);
  if (Values.Num() != 3) {
    return false;
  }
  const FVector Parsed{Values[0]->AsNumber(), Values[1]->AsNumber(),
                       Values[2]->AsNumber()};
  if (!FMath::IsFinite(Parsed.X) || !FMath::IsFinite(Parsed.Y) ||
      !FMath::IsFinite(Parsed.Z)) {
    return false;
  }
  Result = Parsed;
  return true;
}

uvd::Quaternion JsonQuaternion(const TSharedPtr<FJsonObject>& Object,
                               const TCHAR* Name) {
  const TArray<TSharedPtr<FJsonValue>>& Values = Object->GetArrayField(Name);
  return uvd::normalize_quaternion(
      {Values[0]->AsNumber(), Values[1]->AsNumber(), Values[2]->AsNumber(),
       Values[3]->AsNumber()});
}

uvd::QuadraticPolynomial Polynomial(const TSharedPtr<FJsonObject>& Object,
                                    const TCHAR* Name) {
  const TArray<TSharedPtr<FJsonValue>>& Values = Object->GetArrayField(Name);
  return {.x2 = Values[0]->AsNumber(),
          .x1 = Values[1]->AsNumber(),
          .x0 = Values[2]->AsNumber()};
}

uvd::SurfaceMap Surface(const TSharedPtr<FJsonObject>& Object) {
  return {.neutral_rad = Number(Object, TEXT("neutral_rad")),
          .min_rad = Number(Object, TEXT("min_rad")),
          .max_rad = Number(Object, TEXT("max_rad")),
          .direction = Number(Object, TEXT("direction"))};
}

bool ReadJson(const FString& Path, TSharedPtr<FJsonObject>& Root) {
  FString Source;
  if (!FFileHelper::LoadFileToString(Source, *Path)) {
    UE_LOG(LogUvdVehicle, Error, TEXT("Cannot read %s"), *Path);
    return false;
  }
  const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Source);
  if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid()) {
    UE_LOG(LogUvdVehicle, Error, TEXT("Invalid JSON in %s"), *Path);
    return false;
  }
  return true;
}

UMaterialInstanceDynamic* CreateAircraftMaterial(AActor* Owner,
                                                 const FLinearColor& Color) {
  UMaterialInterface* Base = LoadObject<UMaterialInterface>(
      nullptr,
      TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
  if (!Base) {
    return nullptr;
  }
  UMaterialInstanceDynamic* Material =
      UMaterialInstanceDynamic::Create(Base, Owner);
  Material->SetVectorParameterValue(TEXT("Color"), Color);
  return Material;
}

UStaticMeshComponent* AddVisualPart(AActor* Owner, UPrimitiveComponent* Body,
                                    const FName Name, UStaticMesh* Mesh,
                                    const FVector& Location,
                                    const FRotator& Rotation,
                                    const FVector& Scale,
                                    UMaterialInterface* Material) {
  UStaticMeshComponent* Part = NewObject<UStaticMeshComponent>(Owner, Name);
  Part->SetStaticMesh(Mesh);
  Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
  Part->SetGenerateOverlapEvents(false);
  Part->SetCanEverAffectNavigation(false);
  Part->SetupAttachment(Body);
  Part->SetRelativeLocation(Location);
  Part->SetRelativeRotation(Rotation);
  Part->SetRelativeScale3D(Scale);
  if (Material) {
    Part->SetMaterial(0, Material);
  }
  Part->RegisterComponent();
  return Part;
}

bool LoadAircraft(const FString& Path, FUvdVehicleRuntime& Runtime) {
  TSharedPtr<FJsonObject> Root;
  if (!ReadJson(Path, Root)) {
    return false;
  }
  uvd::AerosondeParameters& P = Runtime.AircraftParameters;
  P.model_id = TCHAR_TO_UTF8(*Root->GetStringField(TEXT("model_id")));
  P.mass_kg = Number(Root, TEXT("mass_kg"));

  const TSharedPtr<FJsonObject> Inertia =
      Root->GetObjectField(TEXT("inertia_kgm2"));
  const double Jx = Number(Inertia, TEXT("jx"));
  const double Jy = Number(Inertia, TEXT("jy"));
  const double Jz = Number(Inertia, TEXT("jz"));
  const double Jxz = Number(Inertia, TEXT("jxz"));
  P.inertia_body_kgm2 << Jx, 0.0, -Jxz, 0.0, Jy, 0.0, -Jxz, 0.0, Jz;

  const TSharedPtr<FJsonObject> Geometry =
      Root->GetObjectField(TEXT("geometry"));
  P.wing_area_m2 = Number(Geometry, TEXT("wing_area_m2"));
  P.span_m = Number(Geometry, TEXT("span_m"));
  P.chord_m = Number(Geometry, TEXT("chord_m"));
  P.oswald_efficiency = Number(Geometry, TEXT("oswald_efficiency"));

  const TSharedPtr<FJsonObject> Aero =
      Root->GetObjectField(TEXT("aerodynamics"));
  uvd::AeroDerivatives& C = P.aero;
  C.C_L_0 = Number(Aero, TEXT("CL_0"));
  C.C_L_alpha = Number(Aero, TEXT("CL_alpha"));
  C.C_L_q = Number(Aero, TEXT("CL_q"));
  C.C_L_delta_e = Number(Aero, TEXT("CL_de"));
  C.C_D_parasitic = Number(Aero, TEXT("CD_p"));
  C.C_D_q = Number(Aero, TEXT("CD_q"));
  C.C_D_delta_e = Number(Aero, TEXT("CD_de"));
  C.C_Y_0 = Number(Aero, TEXT("CY0"));
  C.C_Y_beta = Number(Aero, TEXT("CY_beta"));
  C.C_Y_p = Number(Aero, TEXT("CY_p"));
  C.C_Y_r = Number(Aero, TEXT("CY_r"));
  C.C_Y_delta_a = Number(Aero, TEXT("CY_da"));
  C.C_Y_delta_r = Number(Aero, TEXT("CY_dr"));
  C.C_ell_0 = Number(Aero, TEXT("Cl0"));
  C.C_ell_beta = Number(Aero, TEXT("Cl_beta"));
  C.C_ell_p = Number(Aero, TEXT("Cl_p"));
  C.C_ell_r = Number(Aero, TEXT("Cl_r"));
  C.C_ell_delta_a = Number(Aero, TEXT("Cl_da"));
  C.C_ell_delta_r = Number(Aero, TEXT("Cl_dr"));
  C.C_m_0 = Number(Aero, TEXT("Cm0"));
  C.C_m_alpha = Number(Aero, TEXT("Cm_alpha"));
  C.C_m_q = Number(Aero, TEXT("Cm_q"));
  C.C_m_delta_e = Number(Aero, TEXT("Cm_de"));
  C.C_n_0 = Number(Aero, TEXT("Cn0"));
  C.C_n_beta = Number(Aero, TEXT("Cn_beta"));
  C.C_n_p = Number(Aero, TEXT("Cn_p"));
  C.C_n_r = Number(Aero, TEXT("Cn_r"));
  C.C_n_delta_a = Number(Aero, TEXT("Cn_da"));
  C.C_n_delta_r = Number(Aero, TEXT("Cn_dr"));
  C.alpha_stall_rad = Number(Aero, TEXT("alpha0_rad"));
  C.stall_blend_M = Number(Aero, TEXT("lift_blend_M"));

  const TSharedPtr<FJsonObject> Propeller =
      Root->GetObjectField(TEXT("propeller"));
  uvd::PropellerParameters& Prop = P.propeller;
  Prop.diameter_m = Number(Propeller, TEXT("diameter_m"));
  const double Kv = Number(Propeller, TEXT("KV_rpm_per_volt"));
  Prop.motor_torque_constant_Nm_per_A = 60.0 / (Kv * 2.0 * std::numbers::pi);
  Prop.resistance_ohm = Number(Propeller, TEXT("resistance_ohm"));
  Prop.no_load_current_A = Number(Propeller, TEXT("no_load_current_amp"));
  Prop.max_voltage_V = Number(Propeller, TEXT("max_voltage_v"));
  Prop.C_Q = Polynomial(Propeller, TEXT("CQ"));
  Prop.C_T = Polynomial(Propeller, TEXT("CT"));
  Prop.position_body_m = JsonVector(Propeller, TEXT("position_body_m"));
  const TArray<TSharedPtr<FJsonValue>>& Ratio =
      Propeller->GetArrayField(TEXT("advance_ratio_range"));
  Prop.advance_ratio_min = Ratio[0]->AsNumber();
  Prop.advance_ratio_max = Ratio[1]->AsNumber();

  const TSharedPtr<FJsonObject> Actuators =
      Root->GetObjectField(TEXT("actuators"));
  P.actuator = {
      .aileron = Surface(Actuators->GetObjectField(TEXT("aileron"))),
      .elevator = Surface(Actuators->GetObjectField(TEXT("elevator"))),
      .rudder = Surface(Actuators->GetObjectField(TEXT("rudder"))),
  };

  for (const TSharedPtr<FJsonValue>& Value :
       Root->GetArrayField(TEXT("channel_map"))) {
    const TSharedPtr<FJsonObject> Mapping = Value->AsObject();
    const FString Name = Mapping->GetStringField(TEXT("function"));
    EUvdControlFunction Function;
    if (Name == TEXT("aileron")) {
      Function = EUvdControlFunction::Aileron;
    } else if (Name == TEXT("elevator")) {
      Function = EUvdControlFunction::Elevator;
    } else if (Name == TEXT("rudder")) {
      Function = EUvdControlFunction::Rudder;
    } else if (Name == TEXT("throttle")) {
      Function = EUvdControlFunction::Throttle;
    } else {
      return false;
    }
    Runtime.PwmMappings.Add({
        .ChannelIndex =
            static_cast<int32>(Mapping->GetNumberField(TEXT("channel"))) - 1,
        .Function = Function,
        .Calibration =
            {
                .minimum = static_cast<uint16>(
                    Mapping->GetNumberField(TEXT("pwm_min"))),
                .trim = static_cast<uint16>(
                    Mapping->GetNumberField(TEXT("pwm_trim"))),
                .maximum = static_cast<uint16>(
                    Mapping->GetNumberField(TEXT("pwm_max"))),
                .reversed = Mapping->GetBoolField(TEXT("reversed")),
                .throttle = Name == TEXT("throttle"),
            },
    });
  }

  if (Root->HasTypedField<EJson::Object>(TEXT("visual"))) {
    const TSharedPtr<FJsonObject> Visual = Root->GetObjectField(TEXT("visual"));
    Visual->TryGetStringField(TEXT("asset"), Runtime.Visual.AssetPath);
    Visual->TryGetNumberField(TEXT("scale"), Runtime.Visual.UniformScale);
    if (!FMath::IsFinite(Runtime.Visual.UniformScale) ||
        Runtime.Visual.UniformScale <= 0.0) {
      UE_LOG(LogUvdVehicle, Error,
             TEXT("visual.scale must be a finite positive number"));
      return false;
    }
    if (Visual->HasTypedField<EJson::Array>(TEXT("offset_body_m"))) {
      if (!JsonFVector(Visual, TEXT("offset_body_m"),
                       Runtime.Visual.OffsetBodyM)) {
        UE_LOG(LogUvdVehicle, Error,
               TEXT("visual.offset_body_m must contain three finite numbers"));
        return false;
      }
    }
    if (Visual->HasTypedField<EJson::Array>(TEXT("rotation_rpy_deg"))) {
      FVector RollPitchYaw;
      if (!JsonFVector(Visual, TEXT("rotation_rpy_deg"), RollPitchYaw)) {
        UE_LOG(
            LogUvdVehicle, Error,
            TEXT("visual.rotation_rpy_deg must contain three finite numbers"));
        return false;
      }
      Runtime.Visual.Rotation =
          FRotator{RollPitchYaw.Y, RollPitchYaw.Z, RollPitchYaw.X};
    }
  }
  return true;
}

bool LoadOtter(const FString& Path, FUvdVehicleRuntime& Runtime) {
  TSharedPtr<FJsonObject> Root;
  if (!ReadJson(Path, Root)) {
    return false;
  }
  uvd::FossenOtterParameters P;
  P.model_id = TCHAR_TO_UTF8(*Root->GetStringField(TEXT("model_id")));
  P.gravity_mps2 = Number(Root, TEXT("gravity_mps2"));
  P.water_density_kgpm3 = Number(Root, TEXT("water_density_kgpm3"));

  const TSharedPtr<FJsonObject> Hull = Root->GetObjectField(TEXT("hull"));
  P.length_m = Number(Hull, TEXT("length_m"));
  P.beam_m = Number(Hull, TEXT("beam_m"));
  P.hull_mass_kg = Number(Hull, TEXT("mass_kg"));
  P.hull_cg_body_m = JsonVector(Hull, TEXT("cg_body_m"));
  P.hull_radii_of_gyration_m = JsonVector(Hull, TEXT("radii_of_gyration_m"));
  P.maximum_forward_speed_mps = Number(Hull, TEXT("maximum_forward_speed_mps"));

  const TSharedPtr<FJsonObject> Payload = Root->GetObjectField(TEXT("payload"));
  P.payload_mass_kg = Number(Payload, TEXT("mass_kg"));
  P.payload_position_body_m = JsonVector(Payload, TEXT("position_body_m"));

  const TSharedPtr<FJsonObject> Pontoon = Root->GetObjectField(TEXT("pontoon"));
  P.pontoon_beam_m = Number(Pontoon, TEXT("beam_m"));
  P.pontoon_centerline_offset_m = Number(Pontoon, TEXT("centerline_offset_m"));
  P.pontoon_waterline_area_coefficient =
      Number(Pontoon, TEXT("waterline_area_coefficient"));
  P.pontoon_block_coefficient = Number(Pontoon, TEXT("block_coefficient"));
  P.center_of_flotation_x_m = Number(Pontoon, TEXT("center_of_flotation_x_m"));

  const TSharedPtr<FJsonObject> AddedMass =
      Root->GetObjectField(TEXT("added_mass"));
  P.surge_added_mass_coefficient =
      Number(AddedMass, TEXT("surge_soeding_coefficient"));
  P.sway_added_mass_ratio = Number(AddedMass, TEXT("sway_hull_mass_ratio"));
  P.heave_added_mass_ratio = Number(AddedMass, TEXT("heave_hull_mass_ratio"));
  P.roll_added_inertia_ratio = Number(AddedMass, TEXT("roll_inertia_ratio"));
  P.pitch_added_inertia_ratio = Number(AddedMass, TEXT("pitch_inertia_ratio"));
  P.yaw_added_inertia_ratio = Number(AddedMass, TEXT("yaw_inertia_ratio"));

  const TSharedPtr<FJsonObject> Damping = Root->GetObjectField(TEXT("damping"));
  P.sway_time_constant_s = Number(Damping, TEXT("sway_time_constant_s"));
  P.yaw_time_constant_s = Number(Damping, TEXT("yaw_time_constant_s"));
  P.heave_damping_ratio = Number(Damping, TEXT("heave_ratio"));
  P.roll_damping_ratio = Number(Damping, TEXT("roll_ratio"));
  P.pitch_damping_ratio = Number(Damping, TEXT("pitch_ratio"));
  P.nonlinear_yaw_factor = Number(Damping, TEXT("nonlinear_yaw_factor"));

  const TSharedPtr<FJsonObject> Propellers =
      Root->GetObjectField(TEXT("propellers"));
  P.forward_thrust_coefficient =
      Number(Propellers, TEXT("forward_thrust_coefficient"));
  P.reverse_thrust_coefficient =
      Number(Propellers, TEXT("reverse_thrust_coefficient"));
  P.maximum_forward_bollard_force_N =
      Number(Propellers, TEXT("maximum_forward_bollard_force_N"));
  P.maximum_reverse_bollard_force_N =
      Number(Propellers, TEXT("maximum_reverse_bollard_force_N"));
  P.propeller_response_time_s = Number(Propellers, TEXT("response_time_s"));
  try {
    Runtime.OtterModel = uvd::make_fossen_otter_model(P);
  } catch (const std::exception& Error) {
    UE_LOG(LogUvdVehicle, Error, TEXT("Invalid Otter parameters: %s"),
           UTF8_TO_TCHAR(Error.what()));
    return false;
  }
  return true;
}

TArray<TSharedPtr<FJsonValue>> ToJson(const uvd::Vector3& Value) {
  return {MakeShared<FJsonValueNumber>(Value.x()),
          MakeShared<FJsonValueNumber>(Value.y()),
          MakeShared<FJsonValueNumber>(Value.z())};
}

uint16 ReadUint16Le(const uint8* Data) {
  return static_cast<uint16>(Data[0]) |
         static_cast<uint16>(static_cast<uint16>(Data[1]) << 8U);
}

uint32 ReadUint32Le(const uint8* Data) {
  return static_cast<uint32>(Data[0]) | (static_cast<uint32>(Data[1]) << 8U) |
         (static_cast<uint32>(Data[2]) << 16U) |
         (static_cast<uint32>(Data[3]) << 24U);
}

bool OpenSockets(FUvdVehicleRuntime& Runtime) {
  Runtime.ControllerSocket = FUdpSocketBuilder(TEXT("UVD ArduPilot"))
                                 .AsBlocking()
                                 .AsReusable()
                                 .BoundToPort(Runtime.ControllerPort)
                                 .WithReceiveBufferSize(8192)
                                 .WithSendBufferSize(8192);
  Runtime.ControlSocket = FUdpSocketBuilder(TEXT("UVD release"))
                              .AsNonBlocking()
                              .AsReusable()
                              .BoundToPort(Runtime.ControlPort);
  return Runtime.ControllerSocket && Runtime.ControlSocket;
}

void CloseSockets(FUvdVehicleRuntime& Runtime) {
  if (Runtime.ControlSocket) {
    Runtime.ControlSocket->Close();
    ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)
        ->DestroySocket(Runtime.ControlSocket);
    Runtime.ControlSocket = nullptr;
  }
  if (Runtime.ControllerSocket) {
    Runtime.ControllerSocket->Close();
    ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)
        ->DestroySocket(Runtime.ControllerSocket);
    Runtime.ControllerSocket = nullptr;
  }
}

void CommitRelease(FUvdVehicleRuntime& Runtime) {
  Runtime.Released = true;
  Runtime.ReleaseStep = Runtime.Step;
  Runtime.MissedControllerFrames = 0;
  Runtime.DuplicateControllerFrames = 0;
  UE_LOG(LogUvdVehicle, Display,
         TEXT("ArduPilot armed; controller now drives the vehicle"));
}

void PollRelease(FUvdVehicleRuntime& Runtime, double SimTimeS) {
  uint32 Pending = 0;
  while (!Runtime.ReleaseRequested &&
         Runtime.ControlSocket->HasPendingData(Pending)) {
    uint8 Data[32];
    int32 Read = 0;
    TSharedRef<FInternetAddr> Sender =
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
    if (Runtime.ControlSocket->RecvFrom(Data, sizeof(Data), Read, *Sender) &&
        Read >= 7 && FMemory::Memcmp(Data, "release", 7) == 0) {
      Runtime.ReleaseRequested = true;
      if (Runtime.ReleaseParticipants <= 1) {
        CommitRelease(Runtime);
        return;
      }
      FScopeLock Lock(&ReleaseBarrier.Mutex);
      ++ReleaseBarrier.Requested;
      if (ReleaseBarrier.Requested >= Runtime.ReleaseParticipants &&
          ReleaseBarrier.TargetSimTimeS < 0.0) {
        // Both callbacks observe the same future physics time even when their
        // UDP release packets arrive on different ticks.
        ReleaseBarrier.TargetSimTimeS = SimTimeS + 2.0 * Runtime.FixedDtS;
      }
    }
  }
  if (!Runtime.ReleaseRequested || Runtime.Released ||
      Runtime.ReleaseParticipants <= 1) {
    return;
  }
  double TargetSimTimeS = -1.0;
  {
    FScopeLock Lock(&ReleaseBarrier.Mutex);
    TargetSimTimeS = ReleaseBarrier.TargetSimTimeS;
  }
  if (TargetSimTimeS >= 0.0 &&
      SimTimeS + 0.1 * Runtime.FixedDtS >= TargetSimTimeS) {
    CommitRelease(Runtime);
  }
}

bool WriteUtf8(IFileHandle& File, const FString& Text) {
  const FTCHARToUTF8 Utf8(*Text);
  return File.Write(reinterpret_cast<const uint8*>(Utf8.Get()), Utf8.Length());
}

FVector AttitudeDegrees(const uvd::Quaternion& Quaternion) {
  const uvd::Matrix3 Rotation = Quaternion.toRotationMatrix();
  const double Roll = FMath::Atan2(Rotation(2, 1), Rotation(2, 2));
  const double Pitch = FMath::Asin(FMath::Clamp(-Rotation(2, 0), -1.0, 1.0));
  const double Yaw = FMath::Atan2(Rotation(1, 0), Rotation(0, 0));
  constexpr double DegreesPerRadian = 180.0 / std::numbers::pi;
  return {Roll * DegreesPerRadian, Pitch * DegreesPerRadian,
          Yaw * DegreesPerRadian};
}

bool WriteAircraftTruth(FUvdVehicleRuntime& Runtime,
                        const uvd::RigidBodyState& State,
                        const uvd::AircraftEffectorState& Effectors,
                        const uvd::AircraftModelOutput& Model) {
  if (!Runtime.TruthFile) {
    return true;
  }
  const double TimeS = (Runtime.Step - Runtime.ReleaseStep) * Runtime.FixedDtS;
  const FVector Attitude = AttitudeDegrees(State.q_body_to_ned);
  const uvd::AirData& Air = Model.aerodynamics.air_data;
  const FString Row = FString::Printf(
      TEXT("%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,")
          TEXT("%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n"),
      TimeS, State.position_ned_m.x(), State.position_ned_m.y(),
      State.position_ned_m.z(), Attitude.X, Attitude.Y, Attitude.Z,
      State.omega_body_radps.x(), State.omega_body_radps.y(),
      State.omega_body_radps.z(), Air.true_airspeed_mps,
      Air.alpha_rad * 180.0 / std::numbers::pi,
      Air.beta_rad * 180.0 / std::numbers::pi, Effectors.aileron_rad,
      Effectors.elevator_rad, Effectors.rudder_rad, Effectors.throttle);
  if (!WriteUtf8(*Runtime.TruthFile, Row)) {
    return false;
  }
  Runtime.TruthFile->Flush();
  return true;
}

bool WriteOtterTruth(FUvdVehicleRuntime& Runtime,
                     const uvd::OtterModelOutput& Model) {
  if (!Runtime.TruthFile) {
    return true;
  }
  const uvd::OtterState& State = Runtime.OtterState;
  const double TimeS = (Runtime.Step - Runtime.ReleaseStep) * Runtime.FixedDtS;
  const FVector Attitude = AttitudeDegrees(State.body.q_body_to_ned);
  const FString Row = FString::Printf(
      TEXT("%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,")
          TEXT("%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n"),
      TimeS, State.body.position_ned_m.x(), State.body.position_ned_m.y(),
      State.body.position_ned_m.z(), Attitude.X, Attitude.Y, Attitude.Z,
      State.body.velocity_body_mps.x(), State.body.velocity_body_mps.y(),
      State.body.velocity_body_mps.z(), State.body.omega_body_radps.x(),
      State.body.omega_body_radps.y(), State.body.omega_body_radps.z(),
      State.actuators.left_propeller_speed_radps,
      State.actuators.right_propeller_speed_radps, Model.left_thrust_N,
      Model.right_thrust_N,
      Runtime.OtterEnvironment.current_ned_mps.head<2>().norm());
  if (!WriteUtf8(*Runtime.TruthFile, Row)) {
    return false;
  }
  Runtime.TruthFile->Flush();
  return true;
}

bool WriteExchange(FUvdVehicleRuntime& Runtime,
                   const uvd::RigidBodyState& State,
                   const uvd::Vector3& VelocityNed,
                   const uvd::Vector3& SpecificForceBody, double AirspeedMps) {
  if (!Runtime.ExchangeFile || !Runtime.Released) {
    return true;
  }
  const uint64 ReleasedSteps = Runtime.Step - Runtime.ReleaseStep;
  const uint64 FinalStep = static_cast<uint64>(
      FMath::RoundToDouble(Runtime.DurationS / Runtime.FixedDtS));
  if (ReleasedSteps > FinalStep) {
    return true;
  }
  const FVector Attitude = AttitudeDegrees(State.q_body_to_ned);
  const double TimeS = ReleasedSteps * Runtime.FixedDtS;
  const FString Row = FString::Printf(
      TEXT("%.9f,%.9f,%llu,%u,%u,%llu,%llu,")
          TEXT("%.9f,%.9f,%.9f,%.12f,%.12f,%.12f,%.12f,")
              TEXT("%.12f,%.12f,%.12f,%.12f,%.12f,%.12f,")
                  TEXT("%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,")
                      TEXT("%u,%u,%u,%u\n"),
      TimeS, Runtime.PhysicsTimeS, Runtime.PhysicsStep, Runtime.LastFrame,
      Runtime.ControllerFrameRateHz, Runtime.MissedControllerFrames,
      Runtime.DuplicateControllerFrames, Attitude.X, Attitude.Y, Attitude.Z,
      State.q_body_to_ned.w(), State.q_body_to_ned.x(), State.q_body_to_ned.y(),
      State.q_body_to_ned.z(), State.omega_body_radps.x(),
      State.omega_body_radps.y(), State.omega_body_radps.z(),
      SpecificForceBody.x(), SpecificForceBody.y(), SpecificForceBody.z(),
      State.position_ned_m.x(), State.position_ned_m.y(),
      State.position_ned_m.z(), VelocityNed.x(), VelocityNed.y(),
      VelocityNed.z(), AirspeedMps, Runtime.ControllerPwm[0],
      Runtime.ControllerPwm[1], Runtime.ControllerPwm[2],
      Runtime.ControllerPwm[3]);
  if (!WriteUtf8(*Runtime.ExchangeFile, Row)) {
    return false;
  }
  Runtime.ExchangeFile->Flush();
  return true;
}

bool MapCommand(FUvdVehicleRuntime& Runtime, const uint8* Data,
                int32 ChannelCount) {
  if (Runtime.PlantType == EUvdPlantType::FossenOtter) {
    if (ChannelCount <= 2) {
      return false;
    }
    const uvd::PwmCalibration Centered{};
    const double Left = uvd::map_pwm(ReadUint16Le(Data + 8), Centered);
    const double Right = uvd::map_pwm(ReadUint16Le(Data + 8 + 2 * 2), Centered);
    const auto ToShaftSpeed = [&](double Value) {
      return Value >= 0.0
                 ? Value * Runtime.OtterModel.maximum_propeller_speed_radps
                 : Value * -Runtime.OtterModel.minimum_propeller_speed_radps;
    };
    Runtime.OtterControllerCommand = {
        .left_propeller_speed_command_radps = ToShaftSpeed(Left),
        .right_propeller_speed_command_radps = ToShaftSpeed(Right),
    };
    return true;
  }
  for (const FUvdPwmMapping& Mapping : Runtime.PwmMappings) {
    if (Mapping.ChannelIndex >= ChannelCount) {
      return false;
    }
    const double Value = uvd::map_pwm(
        ReadUint16Le(Data + 8 + 2 * Mapping.ChannelIndex), Mapping.Calibration);
    switch (Mapping.Function) {
      case EUvdControlFunction::Aileron:
        Runtime.ControllerCommand.aileron = Value;
        break;
      case EUvdControlFunction::Elevator:
        Runtime.ControllerCommand.elevator = Value;
        break;
      case EUvdControlFunction::Rudder:
        Runtime.ControllerCommand.rudder = Value;
        break;
      case EUvdControlFunction::Throttle:
        Runtime.ControllerCommand.throttle = Value;
        break;
    }
  }
  return true;
}

// DEV NOTE: This follows ArduPilot's JSON SITL backend protocol documented in
// libraries/SITL/examples/JSON/readme.md. Its servo packet carries a frame rate
// and counter; in synchronized mode SITL waits for one returned state per new
// frame. We replay the cached state on retries and count gaps so the exchange
// log can verify that one controller frame advanced with each physics step.
bool ReceiveCommand(FUvdVehicleRuntime& Runtime) {
  while (true) {
    if (!Runtime.ControllerSocket->Wait(ESocketWaitConditions::WaitForRead,
                                        FTimespan::FromSeconds(10.0))) {
      UE_LOG(LogUvdVehicle, Error, TEXT("Timed out waiting for ArduPlane"));
      return false;
    }
    uint8 Data[256];
    int32 Read = 0;
    TSharedRef<FInternetAddr> Sender =
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
    if (!Runtime.ControllerSocket->RecvFrom(Data, sizeof(Data), Read,
                                            *Sender)) {
      return false;
    }
    const int32 Channels = Read == 40 ? 16 : Read == 72 ? 32 : 0;
    const uint16 Magic = Channels == 16 ? 18458 : 29569;
    if (!Channels || ReadUint16Le(Data) != Magic) {
      continue;
    }
    const uint16 FrameRateHz = ReadUint16Le(Data + 2);
    const uint32 Frame = ReadUint32Le(Data + 4);
    if (Runtime.HasFrame && Frame == Runtime.LastFrame) {
      if (!Runtime.IgnoreDuplicateFramesUntilAdvance) {
        ++Runtime.DuplicateControllerFrames;
      }
      if (Runtime.HasReply) {
        int32 Sent = 0;
        Runtime.ControllerSocket->SendTo(Runtime.CachedReply.GetData(),
                                         Runtime.CachedReply.Num(), Sent,
                                         *Runtime.ControllerEndpoint);
      }
      continue;
    }
    if (Runtime.HasFrame) {
      const uint32 FrameDelta = Frame - Runtime.LastFrame;
      if (FrameDelta >= 0x80000000U) {
        UE_LOG(LogUvdVehicle, Error,
               TEXT("ArduPlane controller frame moved backwards: %u -> %u"),
               Runtime.LastFrame, Frame);
        return false;
      }
      if (FrameDelta > 1) {
        Runtime.MissedControllerFrames += FrameDelta - 1;
      }
    }
    Runtime.IgnoreDuplicateFramesUntilAdvance = false;
    if (!MapCommand(Runtime, Data, Channels)) {
      continue;
    }
    Runtime.ControllerFrameRateHz = FrameRateHz;
    for (int32 Channel = 0; Channel < 4; ++Channel) {
      Runtime.ControllerPwm[Channel] = ReadUint16Le(Data + 8 + 2 * Channel);
    }
    Runtime.ControllerEndpoint = Sender;
    Runtime.LastFrame = Frame;
    Runtime.HasFrame = true;
    return true;
  }
}

bool SendState(FUvdVehicleRuntime& Runtime, const uvd::RigidBodyState& State) {
  const uvd::Vector3 VelocityNed =
      State.q_body_to_ned * State.velocity_body_mps;
  const uvd::Vector3 PreviousVelocityNed =
      Runtime.PreviousState.q_body_to_ned *
      Runtime.PreviousState.velocity_body_mps;
  const uvd::Vector3 AccelerationNed =
      (VelocityNed - PreviousVelocityNed) / Runtime.FixedDtS;
  const double GravityMps2 = Runtime.PlantType == EUvdPlantType::FossenOtter
                                 ? Runtime.OtterModel.parameters.gravity_mps2
                                 : uvd::kGravityMps2;
  const uvd::Vector3 SpecificForceBody =
      State.q_body_to_ned.conjugate() *
      (AccelerationNed - uvd::Vector3{0.0, 0.0, GravityMps2});
  double AirspeedMps = 0.0;
  if (Runtime.PlantType == EUvdPlantType::Aerosonde) {
    const uvd::AtmosphereSnapshot Atmosphere =
        uvd::evaluate_isa(Runtime.OriginAltitudeMslM - State.position_ned_m.z(),
                          Runtime.WindNedMps);
    AirspeedMps = uvd::calculate_air_data(State, Atmosphere,
                                          Runtime.AircraftParameters.span_m,
                                          Runtime.AircraftParameters.chord_m)
                      .equivalent_airspeed_mps;
  }

  TSharedPtr<FJsonObject> Imu = MakeShared<FJsonObject>();
  Imu->SetArrayField(TEXT("gyro"), ToJson(State.omega_body_radps));
  Imu->SetArrayField(TEXT("accel_body"), ToJson(SpecificForceBody));
  TSharedPtr<FJsonObject> Json = MakeShared<FJsonObject>();
  Json->SetNumberField(TEXT("timestamp"), Runtime.Step * Runtime.FixedDtS);
  Json->SetObjectField(TEXT("imu"), Imu);
  Json->SetArrayField(TEXT("position"), ToJson(State.position_ned_m));
  Json->SetArrayField(TEXT("quaternion"),
                      {MakeShared<FJsonValueNumber>(State.q_body_to_ned.w()),
                       MakeShared<FJsonValueNumber>(State.q_body_to_ned.x()),
                       MakeShared<FJsonValueNumber>(State.q_body_to_ned.y()),
                       MakeShared<FJsonValueNumber>(State.q_body_to_ned.z())});
  Json->SetArrayField(TEXT("velocity"), ToJson(VelocityNed));
  if (Runtime.PlantType == EUvdPlantType::Aerosonde) {
    Json->SetNumberField(TEXT("airspeed"), AirspeedMps);
  }
  Json->SetBoolField(TEXT("no_time_sync"), false);
  Json->SetBoolField(TEXT("no_lockstep"), false);

  FString Rendered;
  const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>>
      Writer =
          TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(
              &Rendered);
  if (!FJsonSerializer::Serialize(Json.ToSharedRef(), Writer)) {
    return false;
  }
  Rendered = TEXT("\n") + Rendered + TEXT("\n");
  const FTCHARToUTF8 Utf8(*Rendered);
  Runtime.CachedReply.SetNumUninitialized(Utf8.Length());
  FMemory::Memcpy(Runtime.CachedReply.GetData(), Utf8.Get(), Utf8.Length());
  int32 Sent = 0;
  Runtime.HasReply =
      Runtime.ControllerSocket->SendTo(Runtime.CachedReply.GetData(),
                                       Runtime.CachedReply.Num(), Sent,
                                       *Runtime.ControllerEndpoint) &&
      Sent == Runtime.CachedReply.Num();
  return Runtime.HasReply && WriteExchange(Runtime, State, VelocityNed,
                                           SpecificForceBody, AirspeedMps);
}

uvd::RigidBodyState ReadBodyState(const FUvdVehicleRuntime& Runtime,
                                  const FTransform& Transform,
                                  const FVector& VelocityUnrealCmps,
                                  const FVector& OmegaUnrealRadps) {
  const FVector Llh =
      Runtime.Georeference->TransformUnrealPositionToLongitudeLatitudeHeight(
          Transform.GetLocation());
  const uvd::Vector3 PositionNed =
      uvd::ned_from_geodetic(Runtime.Origin, {.latitude_deg = Llh.Y,
                                              .longitude_deg = Llh.X,
                                              .ellipsoid_height_m = Llh.Z});
  const uvd::Matrix3 UnrealBodyToWorld = RotationMatrix(Transform);
  const uvd::Matrix3 BodyToNed =
      NedToUnreal.transpose() * UnrealBodyToWorld * FrdToUnrealBody.transpose();
  const uvd::Vector3 VelocityNed =
      0.01 * NedToUnreal.transpose() * ToUvd(VelocityUnrealCmps);
  const uvd::Vector3 OmegaNed =
      -NedToUnreal.transpose() * ToUvd(OmegaUnrealRadps);
  return {.position_ned_m = PositionNed,
          .q_body_to_ned = uvd::canonicalize(uvd::Quaternion{BodyToNed}),
          .velocity_body_mps = BodyToNed.transpose() * VelocityNed,
          .omega_body_radps = BodyToNed.transpose() * OmegaNed};
}

void CommitKinematicState(FUvdVehicleRuntime& Runtime,
                          Chaos::FRigidBodyHandle_Internal* Handle,
                          const uvd::RigidBodyState& State) {
  // The marine solver is the sole owner of boat state. Chaos retains a
  // kinematic particle only so Unreal's normal physics-to-render handoff moves
  // the root and every attached visual together; it never integrates the boat.
  const uvd::GeodeticPosition Lla =
      uvd::geodetic_from_ned(Runtime.Origin, State.position_ned_m);
  const FVector Position =
      Runtime.Georeference->TransformLongitudeLatitudeHeightPositionToUnreal(
          FVector{Lla.longitude_deg, Lla.latitude_deg, Lla.ellipsoid_height_m});
  const uvd::Matrix3 Rotation =
      BodyToNedRotationToUnreal(State.q_body_to_ned.toRotationMatrix());
  const uvd::Vector3 VelocityNed =
      State.q_body_to_ned * State.velocity_body_mps;
  const uvd::Vector3 VelocityUnreal = 100.0 * NedToUnreal * VelocityNed;
  const uvd::Vector3 OmegaUnreal =
      Rotation * (-FrdToUnrealBody * State.omega_body_radps);
  Handle->SetObjectState(Chaos::EObjectStateType::Kinematic);
  Handle->SetX(Position);
  Handle->SetR(ToUnrealQuaternion(Rotation));
  Handle->SetV(ToUnreal(VelocityUnreal));
  Handle->SetW(ToUnreal(OmegaUnreal));
}

}  // namespace

UUvdVehicleComponent::UUvdVehicleComponent() { bAutoActivate = true; }

UUvdVehicleComponent::~UUvdVehicleComponent() { delete Runtime; }

void UUvdVehicleComponent::BeginPlay() {
  Super::BeginPlay();
  Body = Cast<UPrimitiveComponent>(GetOwner()->GetRootComponent());
  Runtime = new FUvdVehicleRuntime();
  Runtime->OpenLoop = OpenLoop;
  Runtime->ReleaseParticipants = SynchronizedReleaseParticipants;
  if (!Body || !Body->IsSimulatingPhysics() || !LoadConfiguration() ||
      !OpenTruthOutput() || !ConfigureBody() || !ConfigureVisual() ||
      !ConfigureCesium() || (!Runtime->OpenLoop && !OpenSockets(*Runtime))) {
    Runtime->Failed = true;
    UE_LOG(LogUvdVehicle, Error, TEXT("UVD startup failed"));
    return;
  }
  SetInitialState();
  Runtime->PreviousState = Runtime->InitialState;
  RegisterReleaseBarrier(*Runtime);
  if (Runtime->PlantType == EUvdPlantType::FossenOtter && Runtime->OpenLoop) {
    Runtime->Released = true;
  }
  SetAsyncPhysicsTickEnabled(true);
  UE_LOG(
      LogUvdVehicle, Display, TEXT("UVD ready: %s plant%s"),
      Runtime->PlantType == EUvdPlantType::Aerosonde ? TEXT("Aerosonde")
                                                     : TEXT("Fossen Otter"),
      Runtime->OpenLoop ? TEXT(" in open loop") : TEXT(" with ArduPilot JSON"));
}

void UUvdVehicleComponent::EndPlay(const EEndPlayReason::Type EndPlayReason) {
  SetAsyncPhysicsTickEnabled(false);
  if (Runtime) {
    UnregisterReleaseBarrier(*Runtime);
    CloseSockets(*Runtime);
    Runtime->TruthFile.Reset();
    Runtime->ExchangeFile.Reset();
  }
  delete Runtime;
  Runtime = nullptr;
  Super::EndPlay(EndPlayReason);
}

void UUvdVehicleComponent::SetSimulationPaused(bool Paused) {
  if (SimulationPaused == Paused || !Body || !Runtime || Runtime->Failed) {
    return;
  }

  SimulationPaused = Paused;
  if (Runtime->PlantType == EUvdPlantType::FossenOtter) {
    // A paused Otter needs no velocity bookkeeping: disabling this component's
    // async tick freezes the state-owning C++ integrator, while the kinematic
    // Chaos particle keeps its last committed pose available to the camera.
    SetAsyncPhysicsTickEnabled(!Paused);
    if (Paused) {
      Runtime->IgnoreDuplicateFramesUntilAdvance = true;
    }
    return;
  }
  if (Paused) {
    // Freeze only the vehicle and its lockstep exchange. Unreal and Cesium keep
    // rendering normally, so the camera can move without stale temporal frames.
    SetAsyncPhysicsTickEnabled(false);
    Runtime->IgnoreDuplicateFramesUntilAdvance = true;
    PausedLinearVelocity = Body->GetPhysicsLinearVelocity();
    PausedAngularVelocity = Body->GetPhysicsAngularVelocityInRadians();
    Body->SetPhysicsLinearVelocity(FVector::ZeroVector);
    Body->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector);
    Body->PutRigidBodyToSleep();
  } else {
    Body->WakeAllRigidBodies();
    Body->SetPhysicsLinearVelocity(PausedLinearVelocity);
    Body->SetPhysicsAngularVelocityInRadians(PausedAngularVelocity);
    SetAsyncPhysicsTickEnabled(true);
  }
}

bool UUvdVehicleComponent::LoadConfiguration() {
  FString Path = RunConfigPath;
  if (FPaths::IsRelative(Path)) {
    Path = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir(), Path);
  }
  TSharedPtr<FJsonObject> Root;
  if (!ReadJson(Path, Root)) {
    return false;
  }
  Runtime->FixedDtS = Number(Root, TEXT("fixed_dt_s"));

  const TSharedPtr<FJsonObject> Origin = Root->GetObjectField(TEXT("origin"));
  Runtime->OriginAltitudeMslM = Number(Origin, TEXT("altitude_msl_m"));
  Runtime->Origin = {
      .latitude_deg = Number(Origin, TEXT("latitude_deg")),
      .longitude_deg = Number(Origin, TEXT("longitude_deg")),
      .ellipsoid_height_m = uvd::ellipsoid_height_from_msl(
          Runtime->OriginAltitudeMslM,
          Number(Origin, TEXT("geoid_undulation_m"))),
  };
  const TSharedPtr<FJsonObject> Initial =
      Root->GetObjectField(TEXT("initial_state"));
  Runtime->InitialState = {
      .position_ned_m = JsonVector(Initial, TEXT("position_ned_m")),
      .q_body_to_ned = JsonQuaternion(Initial, TEXT("q_body_to_ned")),
      .velocity_body_mps = JsonVector(Initial, TEXT("velocity_body_mps")),
      .omega_body_radps = JsonVector(Initial, TEXT("omega_body_radps")),
  };
  const TSharedPtr<FJsonObject> Controller =
      Root->GetObjectField(TEXT("controller"));
  Runtime->ControllerPort =
      static_cast<uint16>(Number(Controller, TEXT("udp_port")));
  Runtime->ControlPort =
      static_cast<uint16>(Number(Controller, TEXT("control_port")));
  Runtime->TilesetAssetId = static_cast<int64>(
      Number(Root->GetObjectField(TEXT("cesium")), TEXT("tileset_asset_id")));

  if (Root->HasTypedField<EJson::Object>(TEXT("plant"))) {
    const TSharedPtr<FJsonObject> Plant = Root->GetObjectField(TEXT("plant"));
    if (Plant->GetStringField(TEXT("type")) !=
        TEXT("fossen_otter_educational")) {
      UE_LOG(LogUvdVehicle, Error, TEXT("Unsupported plant type"));
      return false;
    }
    Runtime->PlantType = EUvdPlantType::FossenOtter;
    Runtime->DurationS = Root->HasTypedField<EJson::Number>(TEXT("duration_s"))
                             ? Number(Root, TEXT("duration_s"))
                             : 30.0;
    Runtime->OtterState.body = Runtime->InitialState;
    const TArray<TSharedPtr<FJsonValue>>& ShaftSpeed =
        Initial->GetArrayField(TEXT("propeller_speed_radps"));
    Runtime->OtterState.actuators = {ShaftSpeed[0]->AsNumber(),
                                     ShaftSpeed[1]->AsNumber()};
    const TSharedPtr<FJsonObject> Water = Root->GetObjectField(TEXT("water"));
    Runtime->OtterEnvironment = {
        .water_surface_down_m = Number(Water, TEXT("surface_down_m")),
        .current_ned_mps = JsonVector(Water, TEXT("current_ned_mps")),
    };
    const TSharedPtr<FJsonObject> Command =
        Root->GetObjectField(TEXT("command"));
    Runtime->OtterOpenLoopCommand = {
        .left_propeller_speed_command_radps =
            Number(Command, TEXT("left_propeller_speed_radps")),
        .right_propeller_speed_command_radps =
            Number(Command, TEXT("right_propeller_speed_radps")),
    };
    FString PlantPath = Plant->GetStringField(TEXT("parameters"));
    if (FPaths::IsRelative(PlantPath)) {
      PlantPath =
          FPaths::ConvertRelativePathToFull(FPaths::GetPath(Path), PlantPath);
    }
    return LoadOtter(PlantPath, *Runtime);
  }

  Runtime->PlantType = EUvdPlantType::Aerosonde;
  Runtime->DurationS = Root->HasTypedField<EJson::Number>(TEXT("duration_s"))
                           ? Number(Root, TEXT("duration_s"))
                           : 12.0;
  Runtime->WindNedMps = JsonVector(Root, TEXT("wind_ned_mps"));
  const TSharedPtr<FJsonObject> Trim = Root->GetObjectField(TEXT("trim"));
  Runtime->TrimCommand = {
      .aileron = Number(Trim, TEXT("aileron")),
      .elevator = Number(Trim, TEXT("elevator")),
      .rudder = Number(Trim, TEXT("rudder")),
      .throttle = Number(Trim, TEXT("throttle")),
  };
  FString AircraftPath = Root->GetStringField(TEXT("aircraft"));
  if (FPaths::IsRelative(AircraftPath)) {
    AircraftPath =
        FPaths::ConvertRelativePathToFull(FPaths::GetPath(Path), AircraftPath);
  }
  return LoadAircraft(AircraftPath, *Runtime);
}

bool UUvdVehicleComponent::OpenTruthOutput() {
  if (TruthOutputPath.IsEmpty()) {
    return true;
  }
  FString Path = TruthOutputPath;
  if (FPaths::IsRelative(Path)) {
    Path = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir(), Path);
  }
  IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), true);
  Runtime->TruthFile.Reset(
      FPlatformFileManager::Get().GetPlatformFile().OpenWrite(*Path));
  if (!Runtime->TruthFile) {
    UE_LOG(LogUvdVehicle, Error, TEXT("Could not open truth output %s"), *Path);
    return false;
  }
  const FString Header =
      Runtime->PlantType == EUvdPlantType::FossenOtter
          ? TEXT("time_s,north_m,east_m,down_m,roll_deg,pitch_deg,yaw_deg,")
                TEXT("u_mps,v_mps,w_mps,p_radps,q_radps,r_radps,")
                    TEXT("left_propeller_radps,right_propeller_radps,")
                        TEXT("left_thrust_N,right_thrust_N,current_speed_mps\n")
          : TEXT("time_s,north_m,east_m,down_m,roll_deg,pitch_deg,yaw_deg,")
                TEXT("p_radps,q_radps,r_radps,tas_mps,alpha_deg,beta_deg,")
                    TEXT("aileron_rad,elevator_rad,rudder_rad,throttle\n");
  if (!WriteUtf8(*Runtime->TruthFile, Header)) {
    return false;
  }
  Runtime->TruthFile->Flush();
  const FString ExchangePath =
      FPaths::Combine(FPaths::GetPath(Path), TEXT("exchange.csv"));
  Runtime->ExchangeFile.Reset(
      FPlatformFileManager::Get().GetPlatformFile().OpenWrite(*ExchangePath));
  if (!Runtime->ExchangeFile) {
    UE_LOG(LogUvdVehicle, Error, TEXT("Could not open exchange output %s"),
           *ExchangePath);
    return false;
  }
  FString ExchangeHeader =
      TEXT("time_s,physics_time_s,step,servo_frame,frame_rate_hz,")
          TEXT("missed_frames,duplicate_frames,roll_deg,pitch_deg,yaw_deg,")
              TEXT("q_w,q_x,q_y,q_z,p_radps,q_radps,r_radps,")
                  TEXT("ax_mps2,ay_mps2,az_mps2,north_m,east_m,down_m,")
                      TEXT("vn_mps,ve_mps,vd_mps,airspeed_mps,");
  ExchangeHeader += Runtime->PlantType == EUvdPlantType::FossenOtter
                        ? TEXT("left_pwm,unused_pwm,right_pwm,unused_pwm\n")
                        : TEXT("aileron_pwm,elevator_pwm,throttle_pwm,")
                              TEXT("rudder_pwm\n");
  if (!WriteUtf8(*Runtime->ExchangeFile, ExchangeHeader)) {
    return false;
  }
  Runtime->ExchangeFile->Flush();
  UE_LOG(LogUvdVehicle, Display, TEXT("Writing truth to %s"), *Path);
  UE_LOG(LogUvdVehicle, Display, TEXT("Writing lockstep exchange to %s"),
         *ExchangePath);
  return true;
}

bool UUvdVehicleComponent::ConfigureBody() {
  FBodyInstance* Instance = Body->GetBodyInstance();
  if (!Instance) {
    return false;
  }
  if (Runtime->PlantType == EUvdPlantType::FossenOtter) {
    // Keep a physics particle for the async physics-to-render handoff, but make
    // it kinematic and unable to collide. Removing collision entirely can also
    // remove the particle and leave no safe physics-thread pose writer.
    //
    // Unreal kinematics normally flow from the game thread into Chaos. This
    // plant deliberately flows the other way: the fixed-step solver commits a
    // pose on the physics thread, and Chaos publishes that pose to the root
    // component. Without this flag the numerical state advances correctly but
    // the rendered boat remains at its initial transform.
    Instance->SetUpdateKinematicFromSimulation(true);
    Body->SetCollisionEnabled(ECollisionEnabled::PhysicsOnly);
    Body->SetCollisionResponseToAllChannels(ECR_Ignore);
    Body->SetGenerateOverlapEvents(false);
    Body->SetEnableGravity(false);
    Body->SetLinearDamping(0.0F);
    Body->SetAngularDamping(0.0F);
    FPhysicsCommand::ExecuteWrite(
        Instance->GetPhysicsActor(), [&](const FPhysicsActorHandle& Actor) {
          FPhysicsInterface::SetSleepThresholdMultiplier_AssumesLocked(Actor,
                                                                       0.0F);
          FPhysicsInterface::PutToSleep_AssumesLocked(Actor);
        });
    return true;
  }
  Instance->SetMassOverride(Runtime->AircraftParameters.mass_kg, true);
  Instance->UpdateMassProperties();
  const uvd::Matrix3 TargetInertia =
      BodyInertiaToUnreal(Runtime->AircraftParameters.inertia_body_kgm2);
  Eigen::SelfAdjointEigenSolver<uvd::Matrix3> Solver{TargetInertia};
  if (Solver.info() != Eigen::Success) {
    return false;
  }
  uvd::Matrix3 Axes = Solver.eigenvectors();
  if (Axes.determinant() < 0.0) {
    Axes.col(2) *= -1.0;
  }
  const FTransform MassSpace{ToUnrealQuaternion(Axes), FVector::ZeroVector};
  FPhysicsCommand::ExecuteWrite(
      Instance->GetPhysicsActor(), [&](const FPhysicsActorHandle& Actor) {
        FPhysicsInterface::SetMassSpaceInertiaTensor_AssumesLocked(
            Actor, ToUnreal(Solver.eigenvalues()));
        FPhysicsInterface::SetComLocalPose_AssumesLocked(Actor, MassSpace);
        FPhysicsInterface::SetSleepThresholdMultiplier_AssumesLocked(Actor,
                                                                     0.0F);
      });
  Instance->SetInertiaConditioningEnabled(false);
  Body->SetLinearDamping(0.0F);
  Body->SetAngularDamping(0.0F);
  Body->SetEnableGravity(true);
  return true;
}

bool UUvdVehicleComponent::ConfigureVisual() {
  AActor* Owner = GetOwner();
  if (Runtime->PlantType == EUvdPlantType::FossenOtter) {
    UStaticMesh* Cube =
        LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
    UStaticMesh* Cylinder = LoadObject<UStaticMesh>(
        nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    if (!Cube || !Cylinder) {
      return false;
    }
    UMaterialInstanceDynamic* HullMaterial =
        CreateAircraftMaterial(Owner, FLinearColor{0.04F, 0.12F, 0.18F, 1.0F});
    UMaterialInstanceDynamic* DeckMaterial =
        CreateAircraftMaterial(Owner, FLinearColor{0.92F, 0.36F, 0.08F, 1.0F});
    const double LengthCm = 100.0 * Runtime->OtterModel.parameters.length_m;
    const double PontoonWidthCm =
        100.0 * Runtime->OtterModel.parameters.pontoon_beam_m;
    const double OffsetCm =
        100.0 * Runtime->OtterModel.parameters.pontoon_centerline_offset_m;
    const double DraftCm = 100.0 * Runtime->OtterModel.loaded_draft_m;
    for (const double Side : {-1.0, 1.0}) {
      AddVisualPart(
          Owner, Body, Side < 0.0 ? TEXT("LeftPontoon") : TEXT("RightPontoon"),
          Cube, FVector{0.0, Side * OffsetCm, 0.0}, FRotator::ZeroRotator,
          FVector{LengthCm / 100.0, PontoonWidthCm / 100.0,
                  FMath::Max(12.0, DraftCm) / 100.0},
          HullMaterial);
    }
    AddVisualPart(
        Owner, Body, TEXT("OtterDeck"), Cube, FVector{0.0, 0.0, 22.0},
        FRotator::ZeroRotator,
        FVector{0.85 * LengthCm / 100.0, 2.0 * OffsetCm / 100.0, 0.08},
        DeckMaterial);
    AddVisualPart(Owner, Body, TEXT("OtterMast"), Cylinder,
                  FVector{-0.1 * LengthCm, 0.0, 52.0}, FRotator::ZeroRotator,
                  FVector{0.05, 0.05, 0.55}, DeckMaterial);
    UE_LOG(LogUvdVehicle, Display,
           TEXT("Vehicle visual: generic Fossen Otter twin hull"));
    return true;
  }
  if (!Runtime->Visual.AssetPath.IsEmpty()) {
    const FSoftObjectPath AssetPath{Runtime->Visual.AssetPath};
    UStaticMesh* Mesh = Cast<UStaticMesh>(AssetPath.TryLoad());
    if (Mesh) {
      const FVector OffsetUnrealCm =
          100.0 * FVector{Runtime->Visual.OffsetBodyM.X,
                          Runtime->Visual.OffsetBodyM.Y,
                          -Runtime->Visual.OffsetBodyM.Z};
      AddVisualPart(
          Owner, Body, TEXT("AircraftVisual"), Mesh, OffsetUnrealCm,
          Runtime->Visual.Rotation,
          FVector{Runtime->Visual.UniformScale, Runtime->Visual.UniformScale,
                  Runtime->Visual.UniformScale},
          nullptr);
      UE_LOG(LogUvdVehicle, Display, TEXT("Aircraft visual: %s"),
             *Runtime->Visual.AssetPath);
      return true;
    }
    UE_LOG(LogUvdVehicle, Warning,
           TEXT("Cannot load aircraft visual %s; using the generic aircraft"),
           *Runtime->Visual.AssetPath);
  }

  UStaticMesh* Cube =
      LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
  UStaticMesh* Cylinder = LoadObject<UStaticMesh>(
      nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
  UStaticMesh* Sphere = LoadObject<UStaticMesh>(
      nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
  if (!Cube || !Cylinder || !Sphere) {
    UE_LOG(
        LogUvdVehicle, Error,
        TEXT("Cannot load Unreal's built-in meshes for the generic aircraft"));
    return false;
  }

  UMaterialInstanceDynamic* BodyMaterial =
      CreateAircraftMaterial(Owner, FLinearColor{0.025F, 0.08F, 0.18F, 1.0F});
  UMaterialInstanceDynamic* WingMaterial =
      CreateAircraftMaterial(Owner, FLinearColor{0.9F, 0.22F, 0.025F, 1.0F});
  UMaterialInstanceDynamic* AccentMaterial =
      CreateAircraftMaterial(Owner, FLinearColor{0.85F, 0.88F, 0.92F, 1.0F});

  const double SpanCm =
      FMath::Max(100.0, 100.0 * Runtime->AircraftParameters.span_m);
  const double ChordCm =
      FMath::Max(15.0, 100.0 * Runtime->AircraftParameters.chord_m);
  const double FuselageLengthCm = FMath::Max(6.0 * ChordCm, 0.60 * SpanCm);
  const double FuselageDiameterCm = FMath::Max(14.0, 0.06 * SpanCm);
  const double NoseLengthCm = 0.18 * FuselageLengthCm;
  const double WingChordCm = FMath::Max(1.7 * ChordCm, 0.12 * SpanCm);
  const double WingThicknessCm = FMath::Max(3.0, 0.012 * SpanCm);
  const double HalfWingSpanCm = 0.5 * SpanCm;
  const double TailBoomLengthCm = 0.36 * SpanCm;
  const double TailSpanCm = 0.36 * SpanCm;
  const double TailChordCm = FMath::Max(ChordCm, 0.10 * SpanCm);
  const double TailX = -0.62 * SpanCm;

  AddVisualPart(Owner, Body, TEXT("GenericFuselage"), Cylinder,
                FVector::ZeroVector, FRotator{90.0, 0.0, 0.0},
                FVector{FuselageDiameterCm / 100.0, FuselageDiameterCm / 100.0,
                        FuselageLengthCm / 100.0},
                BodyMaterial);
  AddVisualPart(Owner, Body, TEXT("GenericNose"), Sphere,
                FVector{0.5 * FuselageLengthCm, 0.0, 0.0},
                FRotator::ZeroRotator,
                FVector{NoseLengthCm / 100.0, FuselageDiameterCm / 100.0,
                        FuselageDiameterCm / 100.0},
                AccentMaterial);
  AddVisualPart(
      Owner, Body, TEXT("GenericTailBoom"), Cylinder,
      FVector{-0.5 * FuselageLengthCm - 0.42 * TailBoomLengthCm, 0.0, 0.0},
      FRotator{90.0, 0.0, 0.0},
      FVector{0.35 * FuselageDiameterCm / 100.0,
              0.35 * FuselageDiameterCm / 100.0, TailBoomLengthCm / 100.0},
      BodyMaterial);
  AddVisualPart(Owner, Body, TEXT("GenericRightWing"), Cube,
                FVector{0.0, 0.25 * SpanCm, 0.0}, FRotator{0.0, 4.0, 3.0},
                FVector{WingChordCm / 100.0, HalfWingSpanCm / 100.0,
                        WingThicknessCm / 100.0},
                WingMaterial);
  AddVisualPart(Owner, Body, TEXT("GenericLeftWing"), Cube,
                FVector{0.0, -0.25 * SpanCm, 0.0}, FRotator{0.0, -4.0, -3.0},
                FVector{WingChordCm / 100.0, HalfWingSpanCm / 100.0,
                        WingThicknessCm / 100.0},
                WingMaterial);
  AddVisualPart(Owner, Body, TEXT("GenericTailPlane"), Cube,
                FVector{TailX, 0.0, 0.0}, FRotator::ZeroRotator,
                FVector{TailChordCm / 100.0, TailSpanCm / 100.0,
                        0.7 * WingThicknessCm / 100.0},
                WingMaterial);
  AddVisualPart(Owner, Body, TEXT("GenericVerticalTail"), Cube,
                FVector{TailX, 0.0, 0.5 * TailSpanCm * 0.45},
                FRotator::ZeroRotator,
                FVector{TailChordCm / 100.0, 0.7 * WingThicknessCm / 100.0,
                        0.45 * TailSpanCm / 100.0},
                WingMaterial);

  UE_LOG(LogUvdVehicle, Display, TEXT("Aircraft visual: generic fixed wing"));
  return true;
}

bool UUvdVehicleComponent::ConfigureCesium() {
  GetWorld()->GetWorldSettings()->bEnableWorldBoundsChecks = false;
  Runtime->Georeference =
      ACesiumGeoreference::GetDefaultGeoreference(GetWorld());
  if (!Runtime->Georeference) {
    return false;
  }
  if (!ConfigureWorld) {
    return true;
  }
  Runtime->Georeference->SetActorTransform(FTransform::Identity);
  Runtime->Georeference->SetScale(100.0);
  Runtime->Georeference->SetOriginPlacement(
      EOriginPlacement::CartographicOrigin);
  Runtime->Georeference->SetOriginLongitudeLatitudeHeight(
      FVector(Runtime->Origin.longitude_deg, Runtime->Origin.latitude_deg,
              Runtime->Origin.ellipsoid_height_m));

  ACesium3DTileset* Terrain = GetWorld()->SpawnActor<ACesium3DTileset>();
  if (!Terrain) {
    return false;
  }
  Terrain->SetGeoreference(
      TSoftObjectPtr<ACesiumGeoreference>(Runtime->Georeference));
  const FString Token =
      FPlatformMisc::GetEnvironmentVariable(TEXT("CESIUM_ION_TOKEN"));
  if (Token.IsEmpty()) {
    Terrain->SetTilesetSource(ETilesetSource::FromEllipsoid);
    UE_LOG(LogUvdVehicle, Display,
           TEXT("CESIUM_ION_TOKEN is not set; showing the Cesium ellipsoid"));
  } else {
    Terrain->SetTilesetSource(ETilesetSource::FromCesiumIon);
    Terrain->SetIonAccessToken(Token);
    Terrain->SetIonAssetID(Runtime->TilesetAssetId);
  }
  return true;
}

void UUvdVehicleComponent::SetInitialState() {
  const uvd::RigidBodyState& State = Runtime->InitialState;
  const uvd::GeodeticPosition Lla =
      uvd::geodetic_from_ned(Runtime->Origin, State.position_ned_m);
  const FVector Position =
      Runtime->Georeference->TransformLongitudeLatitudeHeightPositionToUnreal(
          FVector(Lla.longitude_deg, Lla.latitude_deg, Lla.ellipsoid_height_m));
  const uvd::Matrix3 Rotation =
      BodyToNedRotationToUnreal(State.q_body_to_ned.toRotationMatrix());
  const uvd::Vector3 VelocityNed =
      State.q_body_to_ned * State.velocity_body_mps;
  const uvd::Vector3 VelocityUnreal = 100.0 * NedToUnreal * VelocityNed;
  const uvd::Vector3 OmegaUnreal =
      Rotation * (-FrdToUnrealBody * State.omega_body_radps);
  Body->SetWorldLocationAndRotation(Position, ToUnrealQuaternion(Rotation),
                                    false, nullptr,
                                    ETeleportType::TeleportPhysics);
  Body->SetPhysicsLinearVelocity(ToUnreal(VelocityUnreal));
  Body->SetPhysicsAngularVelocityInRadians(ToUnreal(OmegaUnreal));
}

void UUvdVehicleComponent::AsyncPhysicsTickComponent(float DeltaTime,
                                                     float SimTime) {
  Super::AsyncPhysicsTickComponent(DeltaTime, SimTime);
  if (!Runtime || Runtime->Failed || !Body) {
    return;
  }
  Runtime->PhysicsTimeS = SimTime;
  Runtime->PhysicsStep =
      static_cast<uint64>(FMath::RoundToDouble(SimTime / Runtime->FixedDtS));
  FBodyInstanceAsyncPhysicsTickHandle Handle =
      Body->GetBodyInstanceAsyncPhysicsTickHandle();
  if (!Handle) {
    return;
  }

  if (Runtime->PlantType == EUvdPlantType::FossenOtter) {
    // The Otter uses a conventional fixed-step numerical plant. A controller
    // frame chooses the command for exactly one RK4 step; the resulting state
    // is then copied to a kinematic Chaos particle for rendering. Keeping this
    // branch separate from the aircraft's force path prevents two solvers from
    // advancing the same vehicle and makes headless and Unreal runs comparable.
    const uvd::RigidBodyState& State = Runtime->OtterState.body;
    if (!uvd::is_finite(State)) {
      Runtime->Failed = true;
      UE_LOG(LogUvdVehicle, Error, TEXT("Otter state became nonfinite"));
      return;
    }
    if (!Runtime->OpenLoop) {
      if (Runtime->HasFrame && !SendState(*Runtime, State)) {
        Runtime->Failed = true;
        return;
      }
      if (!ReceiveCommand(*Runtime)) {
        Runtime->Failed = true;
        return;
      }
      PollRelease(*Runtime, SimTime);
    }
    const uvd::OtterCommand Command =
        Runtime->OpenLoop ? Runtime->OtterOpenLoopCommand
                          : (Runtime->Released ? Runtime->OtterControllerCommand
                                               : uvd::OtterCommand{});
    const uvd::OtterModelOutput Model = uvd::evaluate_fossen_otter(
        Runtime->OtterState, Command, Runtime->OtterEnvironment,
        Runtime->OtterModel);
    if (!Model.valid) {
      Runtime->Failed = true;
      UE_LOG(LogUvdVehicle, Error, TEXT("Otter model became invalid"));
      return;
    }

    if (Runtime->Released) {
      const uint64 TruthInterval = static_cast<uint64>(FMath::Max(
          1.0, FMath::RoundToDouble(1.0 / (20.0 * Runtime->FixedDtS))));
      const uint64 ReleasedSteps = Runtime->Step - Runtime->ReleaseStep;
      const uint64 FinalTruthStep = static_cast<uint64>(
          FMath::RoundToDouble(Runtime->DurationS / Runtime->FixedDtS));
      if (ReleasedSteps <= FinalTruthStep &&
          ReleasedSteps % TruthInterval == 0 &&
          !WriteOtterTruth(*Runtime, Model)) {
        Runtime->Failed = true;
        UE_LOG(LogUvdVehicle, Error, TEXT("Could not write Otter truth"));
        return;
      }
    }

    const uvd::OtterState Next = uvd::step_fossen_otter_rk4(
        Runtime->OtterState, Command, Runtime->OtterEnvironment,
        Runtime->OtterModel, Runtime->FixedDtS);
    if (!uvd::is_finite(Next.body)) {
      Runtime->Failed = true;
      UE_LOG(LogUvdVehicle, Error,
             TEXT("Otter integration produced a nonfinite state"));
      return;
    }
    Runtime->PreviousState = State;
    Runtime->OtterState = Next;
    CommitKinematicState(*Runtime, Handle.operator->(), Next.body);
    ++Runtime->Step;
    return;
  }

  const FTransform Transform{FQuat{Handle->R()}, FVector{Handle->X()}};
  const uvd::RigidBodyState State = ReadBodyState(
      *Runtime, Transform, FVector{Handle->V()}, FVector{Handle->W()});
  if (!uvd::is_finite(State)) {
    Runtime->Failed = true;
    UE_LOG(LogUvdVehicle, Error, TEXT("Vehicle state became nonfinite"));
    return;
  }

  if (Runtime->HasFrame && !SendState(*Runtime, State)) {
    Runtime->Failed = true;
    return;
  }
  if (!ReceiveCommand(*Runtime)) {
    Runtime->Failed = true;
    return;
  }
  PollRelease(*Runtime, SimTime);
  const uvd::AircraftCommand& Command =
      Runtime->Released ? Runtime->ControllerCommand : Runtime->TrimCommand;
  const uvd::AtmosphereSnapshot Atmosphere =
      uvd::evaluate_isa(Runtime->OriginAltitudeMslM - State.position_ned_m.z(),
                        Runtime->WindNedMps);
  const uvd::AircraftEffectorState Effectors =
      uvd::map_command(Command, Runtime->AircraftParameters.actuator);
  const uvd::AircraftModelOutput Model = uvd::evaluate_aerosonde(
      State, Effectors, Atmosphere, Runtime->AircraftParameters);
  if (!Model.valid) {
    Runtime->Failed = true;
    UE_LOG(LogUvdVehicle, Error, TEXT("Vehicle model became invalid"));
    return;
  }

  if (Runtime->Released) {
    const uint64 TruthInterval = static_cast<uint64>(FMath::Max(
        1.0, FMath::RoundToDouble(1.0 / (20.0 * Runtime->FixedDtS))));
    const uint64 ReleasedSteps = Runtime->Step - Runtime->ReleaseStep;
    const uint64 FinalTruthStep = static_cast<uint64>(
        FMath::RoundToDouble(Runtime->DurationS / Runtime->FixedDtS));
    if (ReleasedSteps <= FinalTruthStep && ReleasedSteps % TruthInterval == 0 &&
        !WriteAircraftTruth(*Runtime, State, Effectors, Model)) {
      Runtime->Failed = true;
      UE_LOG(LogUvdVehicle, Error, TEXT("Could not write truth output"));
      return;
    }
  }

  const FVector Force =
      ToUnreal(BodyForceToUnreal(Model.total_wrench.force_body_N));
  const FVector Moment =
      ToUnreal(BodyMomentToUnreal(Model.total_wrench.moment_body_Nm));
  Handle->SetObjectState(Chaos::EObjectStateType::Dynamic);
  Handle->AddForce(Transform.TransformVectorNoScale(Force));
  Handle->AddTorque(Transform.TransformVectorNoScale(Moment));
  Runtime->PreviousState = State;
  ++Runtime->Step;

  const uint64 LogInterval = static_cast<uint64>(
      FMath::Max(1.0, FMath::RoundToDouble(1.0 / Runtime->FixedDtS)));
  if (Runtime->Step % LogInterval == 0) {
    const FVector Llh =
        Runtime->Georeference->TransformUnrealPositionToLongitudeLatitudeHeight(
            Transform.GetLocation());
    const FVector Ned = ToUnreal(State.position_ned_m);
    UE_LOG(LogUvdVehicle, Display, TEXT("LLA %s -> NED %s"), *Llh.ToString(),
           *Ned.ToString());
  }
}

// UnrealBuildTool cannot compile sources outside a module directly.

// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_PUBLIC_RBN100_H_
#define FLATBUFFERS_GENERATED_PUBLIC_RBN100_H_

#include "flatbuffers/flatbuffers.h"

namespace RBN100 {

struct FPoint;
struct FPointBuilder;

struct FPoseTransParm;
struct FPoseTransParmBuilder;

struct Public;
struct PublicBuilder;

enum FRequestType {
  FRequestType_Video = 0,
  FRequestType_ScanQR = 1,
  FRequestType_CreateSence = 2,
  FRequestType_CreateMap_Tag = 3,
  FRequestType_CreateMap_Any = 4,
  FRequestType_CreateMap_Elevator = 5,
  FRequestType_CreateMapStop = 6,
  FRequestType_CreatePoseNode = 7,
  FRequestType_CreateStartNode = 8,
  FRequestType_CreateStandByNode = 9,
  FRequestType_CreatePassNode = 10,
  FRequestType_CreateJMRNode = 11,
  FRequestType_ElevatorInfoInput = 12,
  FRequestType_CreateElevatorOutNode = 13,
  FRequestType_CreateElevatorInNode = 14,
  FRequestType_CreateElevatorTransformNode = 15,
  FRequestType_CreateActionStandBy = 16,
  FRequestType_CreateActionJMR = 17,
  FRequestType_CreateActionStart = 18,
  FRequestType_CreateActionElevatorOut = 19,
  FRequestType_CreateActionElevatorIn = 20,
  FRequestType_CreateActionElevatorTransf = 21,
  FRequestType_SetTaskInfo = 22,
  FRequestType_SetSubTask = 23,
  FRequestType_SetPath = 24,
  FRequestType_Login = 25,
  FRequestType_GetSenceList = 26,
  FRequestType_SetState = 27,
  FRequestType_GetAllNode = 28,
  FRequestType_GetAllPath = 29,
  FRequestType_GetAllMap = 30,
  FRequestType_GetSysTime = 31,
  FRequestType_SetSysTime = 32,
  FRequestType_OpenHatch = 33,
  FRequestType_CloseHatch = 34,
  FRequestType_GetWifi = 35,
  FRequestType_GetBattery = 36,
  FRequestType_ElevatorState = 37,
  FRequestType_TaskStop = 38,
  FRequestType_TaskPause = 39,
  FRequestType_TaskReStart = 40,
  FRequestType_MIN = FRequestType_Video,
  FRequestType_MAX = FRequestType_TaskReStart
};

inline const FRequestType (&EnumValuesFRequestType())[41] {
  static const FRequestType values[] = {
    FRequestType_Video,
    FRequestType_ScanQR,
    FRequestType_CreateSence,
    FRequestType_CreateMap_Tag,
    FRequestType_CreateMap_Any,
    FRequestType_CreateMap_Elevator,
    FRequestType_CreateMapStop,
    FRequestType_CreatePoseNode,
    FRequestType_CreateStartNode,
    FRequestType_CreateStandByNode,
    FRequestType_CreatePassNode,
    FRequestType_CreateJMRNode,
    FRequestType_ElevatorInfoInput,
    FRequestType_CreateElevatorOutNode,
    FRequestType_CreateElevatorInNode,
    FRequestType_CreateElevatorTransformNode,
    FRequestType_CreateActionStandBy,
    FRequestType_CreateActionJMR,
    FRequestType_CreateActionStart,
    FRequestType_CreateActionElevatorOut,
    FRequestType_CreateActionElevatorIn,
    FRequestType_CreateActionElevatorTransf,
    FRequestType_SetTaskInfo,
    FRequestType_SetSubTask,
    FRequestType_SetPath,
    FRequestType_Login,
    FRequestType_GetSenceList,
    FRequestType_SetState,
    FRequestType_GetAllNode,
    FRequestType_GetAllPath,
    FRequestType_GetAllMap,
    FRequestType_GetSysTime,
    FRequestType_SetSysTime,
    FRequestType_OpenHatch,
    FRequestType_CloseHatch,
    FRequestType_GetWifi,
    FRequestType_GetBattery,
    FRequestType_ElevatorState,
    FRequestType_TaskStop,
    FRequestType_TaskPause,
    FRequestType_TaskReStart
  };
  return values;
}

inline const char * const *EnumNamesFRequestType() {
  static const char * const names[42] = {
    "Video",
    "ScanQR",
    "CreateSence",
    "CreateMap_Tag",
    "CreateMap_Any",
    "CreateMap_Elevator",
    "CreateMapStop",
    "CreatePoseNode",
    "CreateStartNode",
    "CreateStandByNode",
    "CreatePassNode",
    "CreateJMRNode",
    "ElevatorInfoInput",
    "CreateElevatorOutNode",
    "CreateElevatorInNode",
    "CreateElevatorTransformNode",
    "CreateActionStandBy",
    "CreateActionJMR",
    "CreateActionStart",
    "CreateActionElevatorOut",
    "CreateActionElevatorIn",
    "CreateActionElevatorTransf",
    "SetTaskInfo",
    "SetSubTask",
    "SetPath",
    "Login",
    "GetSenceList",
    "SetState",
    "GetAllNode",
    "GetAllPath",
    "GetAllMap",
    "GetSysTime",
    "SetSysTime",
    "OpenHatch",
    "CloseHatch",
    "GetWifi",
    "GetBattery",
    "ElevatorState",
    "TaskStop",
    "TaskPause",
    "TaskReStart",
    nullptr
  };
  return names;
}

inline const char *EnumNameFRequestType(FRequestType e) {
  if (flatbuffers::IsOutRange(e, FRequestType_Video, FRequestType_TaskReStart)) return "";
  const size_t index = static_cast<size_t>(e);
  return EnumNamesFRequestType()[index];
}

enum FNodeType {
  FNodeType_PoseNode = 0,
  FNodeType_StartNode = 1,
  FNodeType_StandByNode = 2,
  FNodeType_PassNode = 3,
  FNodeType_JMRNode = 4,
  FNodeType_ElevatorInNode = 5,
  FNodeType_ElevatorOutNode = 6,
  FNodeType_ElevatorTransformNode = 7,
  FNodeType_MIN = FNodeType_PoseNode,
  FNodeType_MAX = FNodeType_ElevatorTransformNode
};

inline const FNodeType (&EnumValuesFNodeType())[8] {
  static const FNodeType values[] = {
    FNodeType_PoseNode,
    FNodeType_StartNode,
    FNodeType_StandByNode,
    FNodeType_PassNode,
    FNodeType_JMRNode,
    FNodeType_ElevatorInNode,
    FNodeType_ElevatorOutNode,
    FNodeType_ElevatorTransformNode
  };
  return values;
}

inline const char * const *EnumNamesFNodeType() {
  static const char * const names[9] = {
    "PoseNode",
    "StartNode",
    "StandByNode",
    "PassNode",
    "JMRNode",
    "ElevatorInNode",
    "ElevatorOutNode",
    "ElevatorTransformNode",
    nullptr
  };
  return names;
}

inline const char *EnumNameFNodeType(FNodeType e) {
  if (flatbuffers::IsOutRange(e, FNodeType_PoseNode, FNodeType_ElevatorTransformNode)) return "";
  const size_t index = static_cast<size_t>(e);
  return EnumNamesFNodeType()[index];
}

enum FMarkerState {
  FMarkerState_Success = 0,
  FMarkerState_ImageIsEmpty = 1,
  FMarkerState_ImageNotGray = 2,
  FMarkerState_TargetNoIdDetected = 3,
  FMarkerState_TargetMoreThanOneIdDetected = 4,
  FMarkerState_TargetMatchNotUnique = 5,
  FMarkerState_TargetOnlyIdDetected = 6,
  FMarkerState_TargetIsNotPlane = 7,
  FMarkerState_TargetNotWorkspaceTooFar = 8,
  FMarkerState_TargetNotWorkspaceTooClose = 9,
  FMarkerState_TargetNotWorkspaceOutOfDegreeRange = 10,
  FMarkerState_TargetNotWorkspaceOutOfDegreeRangeLeft = 11,
  FMarkerState_TargetNotWorkspaceOutOfDegreeRangeRight = 12,
  FMarkerState_TargetWorkspaceTilt = 13,
  FMarkerState_TargetWorkspaceTiltLeft = 14,
  FMarkerState_TargetWorkspaceTiltRight = 15,
  FMarkerState_TargetWorkspacePitch = 16,
  FMarkerState_TargetWorkspacePitchUp = 17,
  FMarkerState_TargetWorkspacePitchDown = 18,
  FMarkerState_TargetWorkspaceSideWay = 19,
  FMarkerState_TargetWorkspaceSideWayLeft = 20,
  FMarkerState_TargetWorkspaceSideWayRight = 21,
  FMarkerState_TargetWorkspaceFlip = 22,
  FMarkerState_Unknown = 23,
  FMarkerState_MIN = FMarkerState_Success,
  FMarkerState_MAX = FMarkerState_Unknown
};

inline const FMarkerState (&EnumValuesFMarkerState())[24] {
  static const FMarkerState values[] = {
    FMarkerState_Success,
    FMarkerState_ImageIsEmpty,
    FMarkerState_ImageNotGray,
    FMarkerState_TargetNoIdDetected,
    FMarkerState_TargetMoreThanOneIdDetected,
    FMarkerState_TargetMatchNotUnique,
    FMarkerState_TargetOnlyIdDetected,
    FMarkerState_TargetIsNotPlane,
    FMarkerState_TargetNotWorkspaceTooFar,
    FMarkerState_TargetNotWorkspaceTooClose,
    FMarkerState_TargetNotWorkspaceOutOfDegreeRange,
    FMarkerState_TargetNotWorkspaceOutOfDegreeRangeLeft,
    FMarkerState_TargetNotWorkspaceOutOfDegreeRangeRight,
    FMarkerState_TargetWorkspaceTilt,
    FMarkerState_TargetWorkspaceTiltLeft,
    FMarkerState_TargetWorkspaceTiltRight,
    FMarkerState_TargetWorkspacePitch,
    FMarkerState_TargetWorkspacePitchUp,
    FMarkerState_TargetWorkspacePitchDown,
    FMarkerState_TargetWorkspaceSideWay,
    FMarkerState_TargetWorkspaceSideWayLeft,
    FMarkerState_TargetWorkspaceSideWayRight,
    FMarkerState_TargetWorkspaceFlip,
    FMarkerState_Unknown
  };
  return values;
}

inline const char * const *EnumNamesFMarkerState() {
  static const char * const names[25] = {
    "Success",
    "ImageIsEmpty",
    "ImageNotGray",
    "TargetNoIdDetected",
    "TargetMoreThanOneIdDetected",
    "TargetMatchNotUnique",
    "TargetOnlyIdDetected",
    "TargetIsNotPlane",
    "TargetNotWorkspaceTooFar",
    "TargetNotWorkspaceTooClose",
    "TargetNotWorkspaceOutOfDegreeRange",
    "TargetNotWorkspaceOutOfDegreeRangeLeft",
    "TargetNotWorkspaceOutOfDegreeRangeRight",
    "TargetWorkspaceTilt",
    "TargetWorkspaceTiltLeft",
    "TargetWorkspaceTiltRight",
    "TargetWorkspacePitch",
    "TargetWorkspacePitchUp",
    "TargetWorkspacePitchDown",
    "TargetWorkspaceSideWay",
    "TargetWorkspaceSideWayLeft",
    "TargetWorkspaceSideWayRight",
    "TargetWorkspaceFlip",
    "Unknown",
    nullptr
  };
  return names;
}

inline const char *EnumNameFMarkerState(FMarkerState e) {
  if (flatbuffers::IsOutRange(e, FMarkerState_Success, FMarkerState_Unknown)) return "";
  const size_t index = static_cast<size_t>(e);
  return EnumNamesFMarkerState()[index];
}

enum FElevatorDoorState {
  FElevatorDoorState_UNKNOWN = 0,
  FElevatorDoorState_OPEN = 1,
  FElevatorDoorState_CLOSE = 2,
  FElevatorDoorState_OPENING = 3,
  FElevatorDoorState_CLOSEING = 4,
  FElevatorDoorState_MIN = FElevatorDoorState_UNKNOWN,
  FElevatorDoorState_MAX = FElevatorDoorState_CLOSEING
};

inline const FElevatorDoorState (&EnumValuesFElevatorDoorState())[5] {
  static const FElevatorDoorState values[] = {
    FElevatorDoorState_UNKNOWN,
    FElevatorDoorState_OPEN,
    FElevatorDoorState_CLOSE,
    FElevatorDoorState_OPENING,
    FElevatorDoorState_CLOSEING
  };
  return values;
}

inline const char * const *EnumNamesFElevatorDoorState() {
  static const char * const names[6] = {
    "UNKNOWN",
    "OPEN",
    "CLOSE",
    "OPENING",
    "CLOSEING",
    nullptr
  };
  return names;
}

inline const char *EnumNameFElevatorDoorState(FElevatorDoorState e) {
  if (flatbuffers::IsOutRange(e, FElevatorDoorState_UNKNOWN, FElevatorDoorState_CLOSEING)) return "";
  const size_t index = static_cast<size_t>(e);
  return EnumNamesFElevatorDoorState()[index];
}

struct FPoint FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  typedef FPointBuilder Builder;
  enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
    VT_X = 4,
    VT_Y = 6
  };
  float x() const {
    return GetField<float>(VT_X, 0.0f);
  }
  float y() const {
    return GetField<float>(VT_Y, 0.0f);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyField<float>(verifier, VT_X) &&
           VerifyField<float>(verifier, VT_Y) &&
           verifier.EndTable();
  }
};

struct FPointBuilder {
  typedef FPoint Table;
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_x(float x) {
    fbb_.AddElement<float>(FPoint::VT_X, x, 0.0f);
  }
  void add_y(float y) {
    fbb_.AddElement<float>(FPoint::VT_Y, y, 0.0f);
  }
  explicit FPointBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  FPointBuilder &operator=(const FPointBuilder &);
  flatbuffers::Offset<FPoint> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<FPoint>(end);
    return o;
  }
};

inline flatbuffers::Offset<FPoint> CreateFPoint(
    flatbuffers::FlatBufferBuilder &_fbb,
    float x = 0.0f,
    float y = 0.0f) {
  FPointBuilder builder_(_fbb);
  builder_.add_y(y);
  builder_.add_x(x);
  return builder_.Finish();
}

struct FPoseTransParm FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  typedef FPoseTransParmBuilder Builder;
  enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
    VT_X = 4,
    VT_Y = 6,
    VT_Q_W = 8,
    VT_Q_X = 10,
    VT_Q_Y = 12,
    VT_Q_Z = 14
  };
  float x() const {
    return GetField<float>(VT_X, 0.0f);
  }
  float y() const {
    return GetField<float>(VT_Y, 0.0f);
  }
  float q_w() const {
    return GetField<float>(VT_Q_W, 0.0f);
  }
  float q_x() const {
    return GetField<float>(VT_Q_X, 0.0f);
  }
  float q_y() const {
    return GetField<float>(VT_Q_Y, 0.0f);
  }
  float q_z() const {
    return GetField<float>(VT_Q_Z, 0.0f);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyField<float>(verifier, VT_X) &&
           VerifyField<float>(verifier, VT_Y) &&
           VerifyField<float>(verifier, VT_Q_W) &&
           VerifyField<float>(verifier, VT_Q_X) &&
           VerifyField<float>(verifier, VT_Q_Y) &&
           VerifyField<float>(verifier, VT_Q_Z) &&
           verifier.EndTable();
  }
};

struct FPoseTransParmBuilder {
  typedef FPoseTransParm Table;
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_x(float x) {
    fbb_.AddElement<float>(FPoseTransParm::VT_X, x, 0.0f);
  }
  void add_y(float y) {
    fbb_.AddElement<float>(FPoseTransParm::VT_Y, y, 0.0f);
  }
  void add_q_w(float q_w) {
    fbb_.AddElement<float>(FPoseTransParm::VT_Q_W, q_w, 0.0f);
  }
  void add_q_x(float q_x) {
    fbb_.AddElement<float>(FPoseTransParm::VT_Q_X, q_x, 0.0f);
  }
  void add_q_y(float q_y) {
    fbb_.AddElement<float>(FPoseTransParm::VT_Q_Y, q_y, 0.0f);
  }
  void add_q_z(float q_z) {
    fbb_.AddElement<float>(FPoseTransParm::VT_Q_Z, q_z, 0.0f);
  }
  explicit FPoseTransParmBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  FPoseTransParmBuilder &operator=(const FPoseTransParmBuilder &);
  flatbuffers::Offset<FPoseTransParm> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<FPoseTransParm>(end);
    return o;
  }
};

inline flatbuffers::Offset<FPoseTransParm> CreateFPoseTransParm(
    flatbuffers::FlatBufferBuilder &_fbb,
    float x = 0.0f,
    float y = 0.0f,
    float q_w = 0.0f,
    float q_x = 0.0f,
    float q_y = 0.0f,
    float q_z = 0.0f) {
  FPoseTransParmBuilder builder_(_fbb);
  builder_.add_q_z(q_z);
  builder_.add_q_y(q_y);
  builder_.add_q_x(q_x);
  builder_.add_q_w(q_w);
  builder_.add_y(y);
  builder_.add_x(x);
  return builder_.Finish();
}

struct Public FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  typedef PublicBuilder Builder;
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           verifier.EndTable();
  }
};

struct PublicBuilder {
  typedef Public Table;
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  explicit PublicBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  PublicBuilder &operator=(const PublicBuilder &);
  flatbuffers::Offset<Public> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<Public>(end);
    return o;
  }
};

inline flatbuffers::Offset<Public> CreatePublic(
    flatbuffers::FlatBufferBuilder &_fbb) {
  PublicBuilder builder_(_fbb);
  return builder_.Finish();
}

inline const RBN100::Public *GetPublic(const void *buf) {
  return flatbuffers::GetRoot<RBN100::Public>(buf);
}

inline const RBN100::Public *GetSizePrefixedPublic(const void *buf) {
  return flatbuffers::GetSizePrefixedRoot<RBN100::Public>(buf);
}

inline bool VerifyPublicBuffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifyBuffer<RBN100::Public>(nullptr);
}

inline bool VerifySizePrefixedPublicBuffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifySizePrefixedBuffer<RBN100::Public>(nullptr);
}

inline void FinishPublicBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<RBN100::Public> root) {
  fbb.Finish(root);
}

inline void FinishSizePrefixedPublicBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<RBN100::Public> root) {
  fbb.FinishSizePrefixed(root);
}

}  // namespace RBN100

#endif  // FLATBUFFERS_GENERATED_PUBLIC_RBN100_H_
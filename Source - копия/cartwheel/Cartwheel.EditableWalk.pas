unit Cartwheel.EditableWalk;

interface

uses
  Utils.Trajectory,
  Cartwheel.Character, Cartwheel.Controllers;

type
  TEditableWalking = class(TIKVMCController)
  strict private
    procedure addControlParams(const JointName: String; kp, kd: Double; tauMax : Double = 1000.0; scale: Double = 1.0); reintroduce;
    procedure addTrajectory(state: TSimBiConState; const JointName: String; const BaseTrajectoryArray: array of Double;
      relToCharFrame: Boolean = False; reverseOnStance: Integer = TTrajectoryComponent.ROS_DONT_REVERSE);
  public
    constructor Create(b: TCharacter);
  end;

implementation

{ TEditableWalking }

constructor TEditableWalking.Create(b: TCharacter);
var
  state: TSimBiConState;
begin
  inherited Create(b, 'Editable walking');

  stanceHipDamping:= 25;
  stanceHipMaxVelocity:= 4;

  addControlParams('root',             1000.0, 200.0,  200.0, 1.0);
  addControlParams('pelvis_lowerback',   75.0,  17.0,  100.0, 1.0);
  addControlParams('lowerback_torso',    75.0,  17.0,  100.0, 1.0);
  addControlParams('torso_head',         10.0,   3.0,  200.0, 1.0);
  addControlParams('lShoulder',          15.0,   5.0,  200.0, 1.0);
  addControlParams('rShoulder',          15.0,   5.0,  200.0, 1.0);
  addControlParams('lElbow',              5.0,   1.0,  200.0, 1.0);
  addControlParams('rElbow',              5.0,   1.0,  200.0, 1.0);
  addControlParams('lHip',              300.0,  35.0,  200.0, 1.0);
  addControlParams('rHip',              300.0,  35.0,  200.0, 1.0);
  addControlParams('lKnee',             300.0,  35.0, 1000.0, 1.0);
  addControlParams('rKnee',             300.0,  35.0, 1000.0, 1.0);
  addControlParams('lAnkle',             50.0,  15.0,  100.0, 1.0);
  addControlParams('rAnkle',             50.0,  15.0,  100.0, 1.0);
  addControlParams('lToeJoint',           2.0,   0.2,  100.0, 1.0);
  addControlParams('rToeJoint',           2.0,   0.2,  100.0, 1.0);

  state:= TSimBiConState.Create;
  // defaults
  state.setTransitionOnFootContact(True);
  state.setStance(TSimBiConState.STATE_REVERSE_STANCE);
  state.Duration:= 0.5;
  // set
  state.name:= 'State 0';
  state.nextStateIndex:= 0;
  state.duration:= 0.6;

  addTrajectory(state, 'STANCE_Knee', [0.0, 0.0]);
  addTrajectory(state, 'SWING_Ankle', [0.0, 0.0], True);
  addTrajectory(state, 'STANCE_Ankle', [0.0, 0.0], True);
  addTrajectory(state, 'SWING_Shoulder', [0.0, 0.0], True);
  addTrajectory(state, 'STANCE_Shoulder', [0.0, 0.0], True);
  addTrajectory(state, 'pelvis_lowerback', [0.0, 0.0], True);
  addTrajectory(state, 'lowerback_torso', [0.0, 0.0], True);
  addTrajectory(state, 'torso_head', [0.0, 0.0], True);

  addTrajectory(state, 'SWING_ToeJoint', [0.0, 0.0]);
  addTrajectory(state, 'STANCE_ToeJoint', [0.0, 0.0]);

  Self.addState(state);
end;

procedure TEditableWalking.addControlParams(const JointName: String; kp, kd, tauMax, scale: Double);
var
  joint: TJoint;
  params: TControlParams;
begin
  joint:= character.getJointByName(JointName);
  params:= TControlParams.Create(joint);
  params.kp:= kp;
  params.kd:= kd;
  params.maxAbsTorque:= tauMax;
  params.scale:= scale;
  inherited addControlParams(params);
end;

procedure TEditableWalking.addTrajectory(state: TSimBiConState; const JointName: String; const BaseTrajectoryArray: array of Double;
  relToCharFrame: Boolean; reverseOnStance: Integer);
var
  baseTraj: TTrajectory1d;
  i: Integer;
  traj: TTrajectory;
  trajComp: TTrajectoryComponent;
begin
  baseTraj:= TTrajectory1d.Create;
  i:= 0;
  while i < Length(BaseTrajectoryArray) do
  begin
    baseTraj.addKnot(BaseTrajectoryArray[i], BaseTrajectoryArray[i+1]);
    Inc(i, 2);
  end;

  traj:= TTrajectory.Create;
  traj.setRelativeToCharacterFrame(relToCharFrame);
  traj.jName:= JointName;

  trajComp:= TTrajectoryComponent.Create;
  trajComp.setReverseOnStance(reverseOnStance);
  trajComp.setBaseTrajectory(baseTraj);
  traj.addTrajectoryComponent(trajComp);

  state.addTrajectory(traj);
end;

end.

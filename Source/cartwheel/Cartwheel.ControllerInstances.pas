unit Cartwheel.ControllerInstances;

interface

uses
  Utils.Trajectory,
  Box2D.Physics2D, Box2D.Physics2DTypes,
  Cartwheel.Character, Cartwheel.Controllers;

type
  TEditableWalking = class(TIKVMCController)
  public
    constructor Create(b: TCharacter);
  end;

  TRunning = class(TSimBiController)
  public
    constructor Create(b: TCharacter);
  end;

implementation

{ TEditableWalking }

{$UNDEF DOUBLE_STANCE}

constructor TEditableWalking.Create(b: TCharacter);
var
  state: TSimBiConState;
begin
  inherited Create(b, 'Editable walking');

  stanceHipDamping:= 25;
  stanceHipMaxVelocity:= 4;

  addControlParams('root',             1000.0, 200.0,  100.0, 1.0);
  addControlParams('pelvis_lowerback',   75.0,  17.0,  100.0, 1.0);
  addControlParams('lowerback_torso',    75.0,  17.0,  100.0, 1.0);
  addControlParams('torso_head',         10.0,   3.0,  200.0, 1.0);
  addControlParams('lShoulder',         300.0,  35.0,  200.0, 1.0);
  addControlParams('rShoulder',         300.0,  35.0,  200.0, 1.0);
  addControlParams('lElbow',            300.0,  35.0,  200.0, 1.0);
  addControlParams('rElbow',            300.0,  35.0,  200.0, 1.0);
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

  addTrajectory(state, 'root', [0.0, 0.0]);
  addTrajectory(state, 'SWING_Hip', []);

  addTrajectory(state, 'SWING_Knee', [0.0, 0.0]);
  {$IFNDEF DOUBLE_STANCE}
  addTrajectory(state, 'STANCE_Knee', [0.0, -0.200, 1.0, -0.2]);
  {$ELSE}
  addTrajectory(state, 'STANCE_Knee', [0.0, 0.0]);
  {$ENDIF}

  {$IFNDEF DOUBLE_STANCE}
  addTrajectory(state, 'SWING_Ankle', [0.0, -0.19, 1.0, 0.56], True);
  addTrajectory(state, 'STANCE_Ankle', [0.0, 0.56, 1.0, -1.19], True);
  {$ELSE}
  addTrajectory(state, 'SWING_Ankle', [0.0, 0.0], True);
  addTrajectory(state, 'STANCE_Ankle', [0.0, 0.0], True);
  {$ENDIF}

  addTrajectory(state, 'SWING_Shoulder', [0.0, 0.0], True);
  addTrajectory(state, 'STANCE_Shoulder', [0.0, 0.0], True);

  addTrajectory(state, 'STANCE_Elbow', [0.0, 0.0]);
  addTrajectory(state, 'SWING_Elbow', [0.0, 0.0]);

  addTrajectory(state, 'pelvis_lowerback', [0.0, 0.0], True);
  addTrajectory(state, 'lowerback_torso', [0.0, 0.0], True);
  addTrajectory(state, 'torso_head', [0.0, 0.0], True);

  {$IFNDEF DOUBLE_STANCE}
  addStrengthTrajectory(
  addTrajectory(state, 'SWING_ToeJoint', [0.0, 0.0]), [0.3, 0.1, 0.5, 0.1, 0.6, 1.0]);
  {$ELSE}
  addTrajectory(state, 'SWING_ToeJoint', [0.0, 0.0]);
  {$ENDIF}
  addTrajectory(state, 'STANCE_ToeJoint', [0.0, 0.0]);

  Self.addState(state);

  scaleGains(1);

  {$IFDEF DOUBLE_STANCE}
  state.setStance(TSimBiConState.STATE_LEFT_STANCE);
  doubleStanceMode:= True;
  {$ENDIF}
end;

{ TRunning }

constructor TRunning.Create(b: TCharacter);
var
  state: TSimBiConState;

  procedure set_mass_moi(const BodyPartName: String; mass, moi: Double);
  var
    arb: TArticulatedRigidBody;
    MassData: Tb2MassData;
  begin
    arb:= b.getARBByName(BodyPartName);
    MassData.mass:= mass;
    MassData.I:= moi;
    MassData.center:= TVector2.From(0, 0);
    arb.Body.SetMassData(MassData);
  end;

begin
  inherited Create(b, 'Running');
  {
  stanceHipDamping:= 25;
  stanceHipMaxVelocity:= 4;
  }
  addControlParams('root',             3000.0, 70.0, 10000.0, 1.0);
  addControlParams('pelvis_lowerback', 1500.0, 100.0, 10000.0, 1.0);
  addControlParams('lowerback_torso',  1000.0, 100.0, 10000.0, 1.0);
  addControlParams('torso_head',        200.0,  20.0, 10000.0, 1.0);
  addControlParams('lShoulder',         100.0,  10.0, 10000.0, 1.0);
  addControlParams('rShoulder',         100.0,  10.0, 10000.0, 1.0);
  addControlParams('lElbow',              5.0,   1.0, 10000.0, 1.0);
  addControlParams('rElbow',              5.0,   1.0, 10000.0, 1.0);
  addControlParams('lHip',              300.0,  35.0, 1000.0, 1.0);
  addControlParams('rHip',              300.0,  35.0, 1000.0, 1.0);
  addControlParams('lKnee',             300.0,  35.0, 1000.0, 1.0);
  addControlParams('rKnee',             300.0,  35.0, 1000.0, 1.0);
  addControlParams('lAnkle',             50.0,  15.0,  100.0, 1.0);
  addControlParams('rAnkle',             50.0,  15.0,  100.0, 1.0);
  addControlParams('lToeJoint',           2.0,   0.2,  100.0, 1.0);
  addControlParams('rToeJoint',           2.0,   0.2,  100.0, 1.0);
               {
  set_mass_moi('pelvis', 12.9, 0.0705);
  set_mass_moi('lowerBack', 22.5, 0.34);
  set_mass_moi('torso', 22.5, 0.34);

  set_mass_moi('head', 5.2, 0.04);

  set_mass_moi('lUpperArm', 2.2, 0.02);
  set_mass_moi('lLowerArm', 1.7, 0.025);
  set_mass_moi('rUpperArm', 2.2, 0.02);
  set_mass_moi('rLowerArm', 1.7, 0.025);

  set_mass_moi('lUpperLeg', 6.6, 0.15);
  set_mass_moi('lLowerLeg', 3.2, 0.055);
  set_mass_moi('rUpperLeg', 6.6, 0.15);
  set_mass_moi('rLowerLeg', 3.2, 0.055);

  set_mass_moi('lFoot', 1.0, 0.007);
  set_mass_moi('lToes', 0.2, 0.002);
  set_mass_moi('rFoot', 1.0, 0.007);
  set_mass_moi('rToes', 0.2, 0.002);
      }

  {
  addControlParams('root',             3000.0, 300.0, 10000.0, 1.0);
  addControlParams('pelvis_lowerback', 1000.0, 100.0, 10000.0, 1.0);
  addControlParams('lowerback_torso',  1000.0, 100.0, 10000.0, 1.0);
  addControlParams('torso_head',        200.0,  20.0, 10000.0, 1.0);
  addControlParams('lShoulder',         100.0,  10.0, 10000.0, 1.0);
  addControlParams('rShoulder',         100.0,  10.0, 10000.0, 1.0);
  addControlParams('lElbow',              5.0,   1.0, 10000.0, 1.0);
  addControlParams('rElbow',              5.0,   1.0, 10000.0, 1.0);
  addControlParams('lHip',              300.0,  30.0, 10000.0, 1.0);
  addControlParams('rHip',              300.0,  30.0, 10000.0, 1.0);
  addControlParams('lKnee',             300.0,  30.0, 10000.0, 1.0);
  addControlParams('rKnee',             300.0,  30.0, 10000.0, 1.0);
  addControlParams('lAnkle',            100.0,  10.0, 10000.0, 1.0);
  addControlParams('rAnkle',            100.0,  10.0, 10000.0, 1.0);
  addControlParams('lToeJoint',          50.0,   5.0, 10000.0, 1.0);
  addControlParams('rToeJoint',          50.0,   5.0, 10000.0, 1.0);
  }

  {
  addControlParams('root',             1000.0, 200.0,  100.0, 1.0);
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
  }

  state:= TSimBiConState.Create;
  // defaults
  state.setTransitionOnFootContact(True);
  state.setStance(TSimBiConState.STATE_REVERSE_STANCE);
  state.Duration:= 0.5;
  // set
  state.name:= 'State 0';
  state.nextStateIndex:= 0;
  state.setTransitionOnFootContact(False);
  state.setStance(TSimBiConState.STATE_REVERSE_STANCE);
  state.duration:= 0.4;

  addTrajectory(state, 'root', [0.0, 0.0]);

  addLinearBalanceFeedback(
  addTrajectory(state, 'SWING_Hip', [0.541806, 0.438308, 0.692308, 0.362199, 0.859532, 0.160317, 0.996656, -0.200194]), 0.0, 0.5);
  addLinearBalanceFeedback(
  addTrajectory(state, 'SWING_Knee', [0.528428, -2.958482, 0.80602, -1.006429, 0.983278, -0.354748]), 0.0, 0.0);
  addTrajectory(state, 'STANCE_Knee', [0.147157, -0.130628, 0.394649, -0.318731, 0.61204, -0.29114, 0.832776, -0.236208, 0.989967, -0.576787]);
  addTrajectory(state, 'SWING_Ankle', [0.020067, -0.809573, 0.197324, 0.510418, 0.488294, 0.518456, 0.75, 0.5, 0.751, 0.15, 1.0, 0.15]);
  addTrajectory(state, 'STANCE_Ankle', [0.354515, 0.354772, 0.625418, -0.764028, 0.749164, -1.163781]);

  addTrajectory(state, 'STANCE_Shoulder', [0.0301, 0.005792, 0.41806, 0.277104, 0.973244, 0.017375]);
  addTrajectory(state, 'SWING_Shoulder', [0.023411, -0.003989, 0.471572, -0.243329]);

  addStrengthTrajectory(
  addTrajectory(state, 'STANCE_Elbow', [0.307692, 2.573897]), [0.307692, 2.135678]);
  addStrengthTrajectory(
  addTrajectory(state, 'SWING_Elbow', [0.013378, 2.665055]), [0.364548, 2.98995]);

  addTrajectory(state, 'pelvis_lowerback', [0.0, 0.0], True);
  addTrajectory(state, 'lowerback_torso', [0.0, 0.0], True);
  addTrajectory(state, 'torso_head', [0.0, 0.0], True);

  addTrajectory(state, 'STANCE_ToeJoint', [0.692308, -0.025126, 0.856187, 1.532663]);
  addTrajectory(state, 'SWING_ToeJoint', [0.38796, -0.014868, 0.826087, 0.431185]);

  Self.addState(state);

  scaleGains(1);
end;

end.

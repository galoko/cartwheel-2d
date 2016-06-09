unit Cartwheel.Controllers;

interface

uses
  System.SysUtils, System.Math, System.Generics.Collections,
  Box2D.Physics2D, Box2D.Physics2DTypes,
  Utils.Trajectory,
  Cartwheel.Character, Cartwheel.TwoLinkIK, Cartwheel.WorldOracle;

const
  LEFT_STANCE = 0;
  RIGHT_STANCE = 1;

  Up : TVector2 = (X: 0; Y: 1);
  Gravity = -9.8;

type
  TController = class
  strict private
    function GetJointCount: Integer; inline;
  protected
    oldTorques, torques: TArray<Double>;
    noOldTorqueInfo: Boolean;
    Name: String;
  public
    character: TCharacter;
    constructor Create(ch: TCharacter; const Name: String);
    property jointCount : Integer read GetJointCount;
    procedure performPreTasks(dt: Double; cfs: TList<TContactPoint>); virtual;
    function performPostTasks(dt: Double; cfs: TList<TContactPoint>) : Boolean; virtual;
    procedure computeTorques(cfs: TList<TContactPoint>); virtual; abstract;
    procedure applyTorques;
    procedure resetTorques;
  end;

  TControlParams = class
  public
    joint: TJoint;
    controlled: Boolean;
    kp, kd, maxAbsTorque, scale, strength: Double;
    qRelExternallyComputed, relToFrame: Boolean;
    constructor Create(joint: TJoint = nil);
    function getJointName : String; inline;
    function getJoint : TJoint; inline;
  end;

  TJointRelative = record
    Orientation, AngularVelocity: Double;
  end;

  TCharacterState = class
  public
    Position, Velocity: TVector2;
    Orientation, AngularVelocity: Double;
    JointRelatives: TArray<TJointRelative>;
    constructor Create(jointCount: Integer);

    function getJointRelativeOrientation(jIndex: Integer) : Double; inline;
    procedure setJointRelativeOrientation(a: Double; jIndex: Integer);

    function getJointRelativeAngVelocity(jIndex: Integer) : Double; inline;
    procedure setJointRelativeAngVelocity(w: Double; jIndex: Integer); inline;
  end;

  TPoseController = class(TController)
  public
    desiredPose: TCharacterState;
    controlParams: TList<TControlParams>;
    constructor Create(ch: TCharacter; const Name: String);
    procedure addControlParams(params: TControlParams); virtual;
    procedure scaleGains(factor: Double); virtual;
    function computePDTorque(aRel, aRelD, wRel, wRelD: Double; cParams: TControlParams) : Double;
    procedure limitTorque(var torque: Double; cParams: TControlParams);
    procedure scaleAndLimitTorque(var torque: Double; cParams: TControlParams);
    procedure computeTorques(cfs: TList<TContactPoint>); override;
  end;

  TSimBiController = class;

  TBalanceFeedback = class
  public
  	function getFeedbackContribution(con: TSimBiController; j: TJoint; phi: Double; const d, v: TVector2) : Double; virtual; abstract;
  end;

  TLinearBalanceFeedback = class(TBalanceFeedback)
  public
    cd, cv, vMin, vMax, dMin, dMax: Double;
    constructor Create;
  	function getFeedbackContribution(con: TSimBiController; j: TJoint; phi: Double; const d, v: TVector2) : Double; override;
  end;

  TTrajectoryComponent = class
  public
    const
      ROS_LEFT = LEFT_STANCE;
      ROS_RIGHT = RIGHT_STANCE;
      ROS_DONT_REVERSE = 100;
    var
      baseTraj, dTrajScale, vTrajScale: TTrajectory1d;
      reverseAngleOnLeftStance, reverseAngleOnRightStance: Boolean;
      bFeedback: TBalanceFeedback;
      offset: Double;
    procedure setReverseOnStance(stance: Integer);
    function getReverseOnStance : Integer;

    procedure setBaseTrajectory(traj: TTrajectory1d); inline;
    function getBaseTrajectory : TTrajectory1d; inline;

    procedure setVTrajScale(traj: TTrajectory1d); inline;
    function getVTrajScale : TTrajectory1d; inline;

    procedure setDTrajScale(traj: TTrajectory1d); inline;
    function getDTrajScale : TTrajectory1d; inline;

    function evaluateTrajectoryComponent(con: TSimBiController; j: TJoint; stance: Integer; phi: Double; const d, v: TVector2;
      bareTrajectory: Boolean = false) : Double;

    function computeFeedback(con: TSimBiController; j: TJoint; phi: double; const d, v: TVector2) : Double;
  end;

  TTrajectory = class
  public
    components: TList<TTrajectoryComponent>;
    leftStanceIndex, rightStanceIndex: Integer;
    jName: String;
    relToCharFrame: Boolean;
    strengthTraj: TTrajectory1d;
    constructor Create;

    procedure setStrengthTrajectory(traj: TTrajectory1d); inline;
    function getStrengthTrajectory : TTrajectory1d; inline;

    procedure setRelativeToCharacterFrame(rel: Boolean = true); inline;
    function isRelativeToCharacterFrame : Boolean; inline;

    procedure addTrajectoryComponent(trajComp_disown: TTrajectoryComponent); inline;
    procedure clearTrajectoryComponents; inline;
    function getTrajectoryComponent(index: Integer) : TTrajectoryComponent; inline;
    function getTrajectoryComponentCount : Integer; inline;

    function evaluateTrajectory(con: TSimBiController; j: TJoint; stance: Integer; phi: Double; const d, v: TVector2;
      bareTrajectory: Boolean = false) : Double;
    function evaluateStrength(phiToUse: Double) : Double; inline;

    function getJointIndex(stance: Integer) : Integer; inline;
  end;

  TSimBiConState = class
  public
    const
      STATE_LEFT_STANCE = LEFT_STANCE;
      STATE_RIGHT_STANCE = RIGHT_STANCE;
      STATE_REVERSE_STANCE = 100;
      STATE_KEEP_STANCE = 101;
    var
      sTraj: TList<TTrajectory>;
      Name: String;
      nextStateIndex: Integer;
      stateTime: Double;
      reverseStance, keepStance: Boolean;
      stateStance: Integer;
      transitionOnFootContact: Boolean;
      minPhiBeforeTransitionOnFootContact, minSwingFootForceForContact: Double;
      dTrajX, vTrajX: TTrajectory1d;
    constructor Create;
    function getStateStance(oldStance: Integer) : Integer; inline;

    function getTrajectoryCount : Integer; inline;
    function getTrajectory(idx: Integer) : TTrajectory; overload; inline;
    function getTrajectory(const name: String) : TTrajectory; overload; inline;

    function needTransition(phi, swingFootVerticalForce, stanceFootVerticalForce: Double) : Boolean;
    procedure setNextStateIndex(nextStateIndex: Integer); inline;
    procedure setTransitionOnFootContact(transition: Boolean = true); inline;
    function getTransitionOnFootContact : Boolean; inline;

    procedure setStance(stanceType: Integer);
    function getStance : Integer; inline;

    property Duration : Double read stateTime write stateTime;

    procedure addTrajectory(traj_disown: TTrajectory); inline;
    procedure clearTrajectories; inline;
  end;

  TSimBiController = class(TPoseController)
  public
    lFoot, rFoot, root: TArticulatedRigidBody;
    lHipIndex, rHipIndex,
    lShoulderIndex, rShoulderIndex,
    lElbowIndex, rElbowIndex: Integer;

    states: TList<TSimBiConState>;
    rootControlParams: TControlParams;

    stanceHipDamping, stanceHipMaxVelocity, rootPredictiveTorqueScale,
    aRootD: Double;

    stance: Integer;
    stanceFoot, swingFoot: TArticulatedRigidBody;
    stanceHipIndex, swingHipIndex,

    FSMStateIndex: Integer;

    comVelocity, comPosition,
    d, v,
    feetMidpoint,
    doubleStanceCOMError: TVector2;

    phi: Double;

    bodyTouchedTheGround: Boolean;

  	startingState, startingStance: Integer;

    procedure scaleGains(factor: Double); override;
    procedure addControlParams(params: TControlParams); overload; override;
    procedure addControlParams(const JointName: String; kp, kd: Double; tauMax : Double = 1000.0; scale: Double = 1.0); reintroduce; overload;
    function addTrajectory(state: TSimBiConState; const JointName: String; const BaseTrajectoryArray: array of Double;
      relToCharFrame: Boolean = False; reverseOnStance: Integer = TTrajectoryComponent.ROS_DONT_REVERSE) : TTrajectory;
    procedure addStrengthTrajectory(traj: TTrajectory; const StrengthTrajectoryArray: array of Double);
    procedure addLinearBalanceFeedback(traj: TTrajectory; cd, cv: Double);

    function getCurrentState : TSimBiConState;
    function getPhiPtr : PDouble; inline;

    procedure setFSMStateTo(index: Integer);
    procedure transitionToState(stateIndex: Integer);
    function getForceOn(rb: TArticulatedRigidBody; cfs: TList<TContactPoint>) : TVector2;
    function getForceOnFoot(foot: TArticulatedRigidBody; cfs: TList<TContactPoint>) : TVector2;
    function haveRelationBetween(rb, whichBody: TArticulatedRigidBody) : Boolean;
    function isFoot(rb: TArticulatedRigidBody) : Boolean;
    function isStanceFoot(rb: TArticulatedRigidBody) : Boolean;
    function isSwingFoot(rb: TArticulatedRigidBody) : Boolean;
    function getStanceFootWeightRatio(cfs: TList<TContactPoint>) : Double;
    procedure computeHipTorques(aRootD, stanceHipToSwingHipRatio, ffRootTorque: Double);
    procedure blendOutTorques;
    procedure resolveJoints(state: TSimBiConState);
    procedure setStance(newStance: Integer);
    function getState(idx: Integer) : TSimBiConState; inline;
    function getStateCount : Integer; inline;
    procedure addState(state_disown: TSimBiConState); inline;
    procedure clearStates; inline;
    procedure computeTorques(cfs: TList<TContactPoint>); override;
    procedure evaluateJointTargets;
    procedure computePDTorques(cfs: TList<TContactPoint>);
    function performPostTasks(dt: Double; cfs: TList<TContactPoint>) : Boolean; override;
    function advanceInTime(dt: Double; cfs: TList<TContactPoint>) : Integer; virtual;
    property isBodyInContactWithTheGround : Boolean read bodyTouchedTheGround;
    property Phase : Double read phi write phi;
    function getStanceFootPos : TVector2; inline;
    function getSwingFootPos : TVector2; inline;
    function getFSMState : Integer; inline;
    procedure updateDAndV;
    procedure computeD0(phi: Double; var d0: TVector2); inline;
    procedure computeV0(phi: Double; var v0: TVector2); inline;
    class procedure computeDorV(phi: Double; trajX: TTrajectory1d; stance: Integer; var result: TVector2); static;

    constructor Create(b: TCharacter; const Name: String = 'UnnamedSimBiController');
    destructor Destroy; override;
  end;

  TVirtualModelController = class(TController)
  public
    procedure computeTorques(cfs: TList<TContactPoint>); override;
    procedure computeJointTorquesEquivalentToForce(start: TJoint; const pLocal, fGlobal: TVector2; _end: TJoint);
  end;

  TIKVMCController = class;

  TBehaviourController = class
  public
    bip: TCharacter;
    lowLCon: TIKVMCController;
    wo: TWorldOracle;
    swingFootStartPos: TVector2;
    legLength, ankleBaseHeight,
    ubSagittalLean, velDSagittal, kneeBend,
    stepTime, stepHeight: Double;
    procedure setUpperBodyPose(leanSagittal: Double);
    procedure setKneeBend(v: Double; swingAlso : Boolean = false);
    procedure setVelocities(velDS: Double); inline;

    constructor Create(b: TCharacter; llc: TIKVMCController; w: TWorldOracle = nil);
    destructor Destroy; override;

    procedure adjustStepHeight;

    procedure setElbowAngles(leftElbowAngle, rightElbowAngle: Double);
	  procedure setShoulderAngles(leftSwing, rightSwing: Double);

    procedure requestStepTime(stepTime: Double); inline;
    procedure requestStepHeight(stepHeight: Double); inline;
    procedure requestVelocities(velDS: Double); inline;
    procedure requestUpperBodyPose(leanS: Double); inline;
    procedure requestKneeBend(kb: Double); inline;

    property DesiredStepTime : Double read stepTime;
    property DesiredVelocitySagittal : Double read velDSagittal;

    procedure setDesiredSwingFootLocation;

    function computeSwingFootLocationEstimate(const comPos: TVector2; phase: Double) : TVector2;

    procedure initializeDefaultParameters; inline;

    procedure simStepPlan(dt: Double);

    procedure conTransitionPlan;

    function getPanicLevel : Double; inline;
  end;

  TIKVMCController = class(TSimBiController)
  public
    var
      swingKneeIndex: Integer;
      swingKnee, swingHip: TJoint;
      lKneeIndex, rKneeIndex, lAnkleIndex, rAnkleIndex,
      stanceAnkleIndex, stanceKneeIndex, swingAnkleIndex: Integer;
      vmc: TVirtualModelController;
      ffRootTorque: Double;
      behaviour: TBehaviourController;
      velDSagittal: Double;
      swingFootTrajectorySagittal, swingFootHeightTrajectory,
      swingFootTrajectoryDeltaSagittal, swingFootTrajectoryDeltaHeight: TTrajectory1d;
      doubleStanceMode: Boolean;
      comOffsetSagittal, panicHeight, unplannedForHeight: Double;
      armsTarget: TVector2;

    constructor Create(b: TCharacter; const Name: String);

    procedure setBehaviour(behaviour_disown: TBehaviourController); inline;
    
    function computeIPStepLocation : TVector2;
    procedure computeIKSwingLegTargets(dt: Double);
    procedure computeIKArmsTargets(dt: Double);
    procedure DebugComputeIKSwingLegTargets(const Point: TVector2);
    procedure computeTorques(cfs: TList<TContactPoint>); override;
    function computeSwingFootDelta(phiToUse: Double = -1; stanceToUse : Integer = -1) : TVector2;
    function getSwingFootTargetLocation(t: Double; const com: TVector2) : TVector2;
    procedure computeGravityCompensationTorques;
    procedure updateSwingAndStanceReferences;
  	procedure computeIKQandW(parentJIndex, childJIndex: Integer; const parentAxis: TVector2; parentNormal, childNormal: Double; 
      const childEndEffector, wP: TVector2; computeAngVelocities: Boolean; const futureWP: TVector2; dt: Double);
    procedure bubbleUpTorques;
  	procedure computeLegTorques(ankleIndex, kneeIndex, hipIndex: Integer; cfs: TList<TContactPoint>);
    procedure COMJT(cfs: TList<TContactPoint>);
    function computeVirtualForce : TVector2;
    procedure preprocessAnkleVTorque(ankleJointIndex: Integer; cfs: TList<TContactPoint>; var ankleVTorque: Double);
    procedure performPreTasks(dt: Double; cfs: TList<TContactPoint>); override;
    function performPostTasks(dt: Double; cfs: TList<TContactPoint>) : Boolean; override;
    procedure getForceInfoOn(rb: TArticulatedRigidBody; cfs: TList<TContactPoint>; var heelForce, toeForce: Boolean);
  end;

implementation

function clamp(value, min, max: Double) : Double; inline;
begin
  if value < min then
    Result:= min
  else
  if value > max then
    Result:= max
  else Result:= value;
end;

{ TController }

constructor TController.Create(ch: TCharacter; const Name: String);
begin
  Self.Name:= Name;
	Self.character:= ch;
  SetLength(torques, ch.JointCount);
  SetLength(oldTorques, ch.JointCount);
  noOldTorqueInfo:= True;
end;

function TController.GetJointCount: Integer;
begin
  Result:= Length(torques);
end;

procedure TController.performPreTasks(dt: Double; cfs: TList<TContactPoint>);
begin
  computeTorques(cfs);
  applyTorques;
end;

function TController.performPostTasks(dt: Double; cfs: TList<TContactPoint>): Boolean;
begin
  Result:= False;
end;

procedure TController.applyTorques;
const
  maxTorqueRateOfChange = 2000;
var
  i: Integer;
  deltaT, tmpTorque: Double;
begin
  if noOldTorqueInfo then
  begin
    Move(Pointer(torques)^, Pointer(oldTorques)^, Length(oldTorques) * SizeOf(Double));
    noOldTorqueInfo:= False;
  end;

	for i:=0 to jointCount - 1 do
  begin
		deltaT:= torques[i] - oldTorques[i];

    deltaT:= clamp(deltaT, -maxTorqueRateOfChange, maxTorqueRateOfChange);

		tmpTorque:= oldTorques[i] + deltaT;

		character.getJoint(i).setTorque(tmpTorque);
		oldTorques[i]:= tmpTorque;
	end;
end;

procedure TController.resetTorques;
begin
	noOldTorqueInfo:= True;
  FillChar(Pointer(torques)^, Length(torques) * SizeOf(Double), 0);
end;

{ TControlParams }

constructor TControlParams.Create(joint: TJoint);
begin
  Self.joint:= joint;
  strength:= 1;
end;

function TControlParams.getJointName: String;
begin
  if Assigned(joint) then	Exit(joint.Name);
  Result:= 'root';
end;

function TControlParams.getJoint: TJoint;
begin
  Result:= joint;
end;

{ TPoseController }

constructor TPoseController.Create(ch: TCharacter; const Name: String);
var
  i: Integer;
begin
  inherited;
  desiredPose:= TCharacterState.Create(ch.JointCount);
  controlParams:= TList<TControlParams>.Create;
  for i := 0 to jointCount - 1 do
    controlParams.Add(TControlParams.Create(ch.getJoint(i)));
end;

procedure TPoseController.addControlParams(params: TControlParams);
var
  jIndex: Integer;
begin
  jIndex:= character.getJointIndex(params.getJoint);
  if jIndex < 0 then
    raise Exception.CreateFmt('Cannot find joint ''%s'' in character', [params.getJointName]);
  controlParams[jIndex]:= params;
end;

function TPoseController.computePDTorque(aRel, aRelD, wRel, wRelD: Double; cParams: TControlParams): Double;
var
  torque: Double;
begin
  torque:= ((-cParams.kp * (aRelD - aRel)) + (-cParams.kd * (wRelD - wRel))) * cParams.strength;
  scaleAndLimitTorque(torque, cParams);
  Result:= torque;
end;

procedure TPoseController.limitTorque(var torque: Double; cParams: TControlParams);
begin
  torque:= clamp(torque, -cParams.scale * cParams.maxAbsTorque, cParams.scale * cParams.maxAbsTorque);
end;

procedure TPoseController.scaleAndLimitTorque(var torque: Double; cParams: TControlParams);
begin
  torque:= torque * cParams.scale;
  limitTorque(torque, cParams);
end;

procedure TPoseController.scaleGains(factor: Double);
var
  i: Integer;
  kp, kd: Double;
begin
  for i := 0 to controlParams.Count - 1 do
  begin
    if character.getJoint(i).Name.EndsWith('ToeJoint') then
      continue;
    kp:= controlParams[i].Kp * factor;
    controlParams[i].Kp:= kp;
    kd:= Sqrt(kp) * 2;
    controlParams[i].Kd:= kd;
  end;
end;

procedure TPoseController.computeTorques(cfs: TList<TContactPoint>);
var
  rs: TCharacterState;
  i: Integer;
  cParams: TControlParams;
  joint: TJoint;
  parentRB, childRB: TRigidBody;
  parentAworld, frameAworld, frameAngularVelocityInFrame,
  currentOrientationInFrame, desiredOrientationInFrame,
  currentAngularVelocityInFrame, desiredRelativeAngularVelocityInFrame, currentRelativeAngularVelocityInFrame: Double;
begin
  rs:= desiredPose;

  for i:=0 to jointCount - 1 do
  begin
    cParams:= controlParams[i];
    if cParams.controlled then
    begin
      joint:= character.getJoint(i);
			parentRB:= joint.Parent;
			childRB:= joint.Child;
			parentAworld:= -parentRB.getOrientation;
			if not cParams.relToFrame then
      begin
				frameAworld:= parentAworld;
				frameAngularVelocityInFrame:= parentRB.getAngularVelocity;
			end
      else
      begin
				frameAworld:= -0;
				frameAngularVelocityInFrame:= 0;
			end;

			currentOrientationInFrame:= frameAworld + childRB.getOrientation();
			desiredOrientationInFrame:= rs.getJointRelativeOrientation(i);
      
			if cParams.relToFrame then
        desiredOrientationInFrame:= desiredOrientationInFrame + character.GetRevertAngle;

			currentAngularVelocityInFrame:= childRB.getAngularVelocity;
			desiredRelativeAngularVelocityInFrame:= rs.getJointRelativeAngVelocity(i);
			currentRelativeAngularVelocityInFrame:= currentAngularVelocityInFrame - frameAngularVelocityInFrame;

			torques[i]:= computePDTorque(currentOrientationInFrame, desiredOrientationInFrame, currentRelativeAngularVelocityInFrame,
        desiredRelativeAngularVelocityInFrame, cParams);
    end
    else
			torques[i]:= 0;
  end;
end;

{ TCharacterState }

constructor TCharacterState.Create(jointCount: Integer);
begin
  SetLength(Self.JointRelatives, jointCount);
end;

function TCharacterState.getJointRelativeOrientation(jIndex: Integer): Double;
begin
  Result:= JointRelatives[jIndex].Orientation;
end;

procedure TCharacterState.setJointRelativeOrientation(a: Double; jIndex: Integer);
begin
  JointRelatives[jIndex].Orientation:= a;
end;

function TCharacterState.getJointRelativeAngVelocity(jIndex: Integer): Double;
begin
  Result:= JointRelatives[jIndex].AngularVelocity;
end;

procedure TCharacterState.setJointRelativeAngVelocity(w: Double; jIndex: Integer);
begin
  JointRelatives[jIndex].AngularVelocity:= w;
end;

{ TSimBiController }

constructor TSimBiController.Create(b: TCharacter; const Name: String);
var
  lHip, rHip: TJoint;
begin
  inherited;
  states:= TList<TSimBiConState>.Create;

  if b = nil then
		raise Exception.Create('Cannot create a SIMBICON controller if there is no associated biped!!');
	//characters controlled by a simbicon controller are assumed to have: 2 feet
	lFoot:= b.getARBByName('lFoot');
	rFoot:= b.getARBByName('rFoot');

	if (rFoot = nil) or (lFoot = nil) then
  begin
		lFoot:= b.getARBByName('lFoot2');
		rFoot:= b.getARBByName('rFoot2');
	end;

	if (rFoot = nil) or (lFoot = nil) then
		raise Exception.Create('The biped must have the rigid bodies lFoot and rFoot!');

	//and two hips connected to the root
	lHip:= b.getJointByName('lHip');
	rHip:= b.getJointByName('rHip');

	lHipIndex:= b.getJointIndex('lHip');
	rHipIndex:= b.getJointIndex('rHip');

	if (rFoot = nil) or (lFoot = nil) then
		raise Exception.Create('The biped must have the joints lHip and rHip!');

	root:= b.Root;

	if (lHip.Parent <> rHip.Parent) or (lHip.Parent <> root) then
		raise Exception.Create('The biped''s hips must have a common parent, which should be the root of the figure!');

  lShoulderIndex:= b.getJointIndex('lShoulder');
  rShoulderIndex:= b.getJointIndex('rShoulder');
  lElbowIndex:= b.getJointIndex('lElbow');
  rElbowIndex:= b.getJointIndex('rElbow');

	setStance(LEFT_STANCE);
	phi:= 0;

	setFSMStateTo(-1);

	stanceHipDamping:= -1;
	stanceHipMaxVelocity:= 4;
	rootPredictiveTorqueScale:= 0;

	bodyTouchedTheGround:= false;

	startingState:= 0;
	startingStance:= LEFT_STANCE;
end;

destructor TSimBiController.Destroy;
begin

  inherited;
end;

procedure TSimBiController.scaleGains(factor: Double);
begin
  rootControlParams.Kp:= rootControlParams.Kp * factor;
  rootControlParams.Kd:= rootControlParams.Kd * factor;
  // rootControlParams.Kd:= Sqrt(rootControlParams.Kp) * 2;
  inherited;
end;

procedure TSimBiController.addControlParams(params: TControlParams);
begin
  if params.Joint = nil then
    rootControlParams:= params
  else
    inherited;
end;

function TSimBiController.getCurrentState: TSimBiConState;
begin
  if FSMStateIndex < 0 then
    setFSMStateTo(startingState);
  Result:= getState(FSMStateIndex);
end;

function TSimBiController.getPhiPtr: PDouble;
begin
  Result:= @phi;
end;

procedure TSimBiController.setFSMStateTo(index: Integer);
begin
	if index < 0 then
		FSMStateIndex:= -1
	else
  if index >= states.Count then
		FSMStateIndex:= states.Count - 1
	else
		FSMStateIndex:= index;
end;

procedure TSimBiController.transitionToState(stateIndex: Integer);
begin
	setFSMStateTo(stateIndex);
	setStance(states[FSMStateIndex].getStateStance(Self.stance));
	Self.phi:= 0;
end;

function TSimBiController.getForceOn(rb: TArticulatedRigidBody; cfs: TList<TContactPoint>): TVector2;
var
  fNet: TVector2;
  i: Integer;
begin
	fNet:= TVector2.From(0, 0);
	for i:=0 to cfs.Count - 1 do
  begin
		if cfs[i].rb1 = rb then
			fNet:= fNet + cfs[i].f;
		if cfs[i].rb2 = rb then
			fNet:= fNet - cfs[i].f;
	end;
	Result:= fNet;
end;

function TSimBiController.getForceOnFoot(foot: TArticulatedRigidBody; cfs: TList<TContactPoint>): TVector2;
var
  fNet: TVector2;
  i: Integer;
begin
	fNet:= getForceOn(foot, cfs);

	//we will also look at all children of the foot that is passed in (to take care of toes).
	for i:=0 to foot.childJoints.Count - 1 do
		fNet:= fNet + getForceOn(foot.childJoints[i].child, cfs);

	Result:= fNet;
end;

function TSimBiController.haveRelationBetween(rb, whichBody: TArticulatedRigidBody): Boolean;
var
  j: Integer;
begin
	//check against the feet
	if rb = whichBody then
		Exit(true);
	for j:=0 to whichBody.childJoints.Count - 1 do
		if whichBody.childJoints[j].child = rb then
			Exit(true);
	Exit(false);
end;

function TSimBiController.isFoot(rb: TArticulatedRigidBody): Boolean;
begin
  Exit(haveRelationBetween(rb, lFoot) or haveRelationBetween(rb, rFoot));
end;

function TSimBiController.isStanceFoot(rb: TArticulatedRigidBody): Boolean;
begin
  Exit(haveRelationBetween(rb, stanceFoot));
end;

function TSimBiController.isSwingFoot(rb: TArticulatedRigidBody): Boolean;
begin
  Exit(haveRelationBetween(rb, swingFoot));
end;

function TSimBiController.getStanceFootWeightRatio(cfs: TList<TContactPoint>): Double;
var
  stanceFootForce, swingFootForce: TVector2;
  totalYForce: Double;
begin
	stanceFootForce:= getForceOnFoot(stanceFoot, cfs);
	swingFootForce:= getForceOnFoot(swingFoot, cfs);
	totalYForce:= b2Dot(stanceFootForce + swingFootForce, Up);

	if SameValue(totalYForce, 0) then
		Exit(-1)
	else
		Exit(b2Dot(stanceFootForce, Up) / totalYForce);
end;

procedure TSimBiController.computeHipTorques(aRootD, stanceHipToSwingHipRatio, ffRootTorque: Double);
var
  rootTorque, swingHipTorque, aRootDW, rootStrength, rootMakeupTorque,
  stanceHipTorque, wRel, wRelLen: Double;
  i: Integer;
begin
	if stanceHipToSwingHipRatio < 0 then
		rootControlParams.strength:= 0;

	aRootDW:= 0 + aRootD + character.GetRevertAngle;

	rootStrength:= clamp(rootControlParams.strength, 0, 1);

	rootControlParams.strength:= 1;

	//so this is the net torque that the root wants to see, in world coordinates
	rootTorque:= computePDTorque(root.getOrientation, aRootDW, root.getAngularVelocity, 0, rootControlParams);

	rootTorque:= rootTorque + ffRootTorque;

	//we need to compute the total torque that needs to be applied at the hips so that the final net torque on the root is rootTorque
	rootMakeupTorque:= 0;
	for i:=0 to jointCount - 1 do
		if character.getJoint(i).Parent = root then
			rootMakeupTorque:= rootMakeupTorque - torques[i];
	rootMakeupTorque:= rootMakeupTorque - rootTorque;

	//assume the stance foot is in contact...
	stanceHipTorque:= torques[stanceHipIndex];
	swingHipTorque:= torques[swingHipIndex];

	//now, based on the ratio of the forces the feet exert on the ground, and the ratio of the forces between the two feet, we will compute the forces that need to be applied
	//to the two hips, to make up the root makeup torque - which means the final torque the root sees is rootTorque!
	stanceHipTorque:= stanceHipTorque + rootMakeupTorque * stanceHipToSwingHipRatio * rootStrength;
	swingHipTorque:= swingHipTorque + rootMakeupTorque * (1-stanceHipToSwingHipRatio) * rootStrength;

	if stanceHipDamping > 0 then
  begin
		wRel:= root.getAngularVelocity() - character.joints[stanceHipIndex].child.getAngularVelocity();
		wRelLen:= Abs(wRel);
		if wRelLen > stanceHipMaxVelocity then wRel:= wRel * (stanceHipMaxVelocity/wRelLen);
		stanceHipTorque:= stanceHipTorque - wRel * (stanceHipDamping * wRelLen);
	end;

	limitTorque(stanceHipTorque, controlParams[stanceHipIndex]);

	limitTorque(swingHipTorque, controlParams[swingHipIndex]);

	//and done...
	torques[stanceHipIndex]:= stanceHipTorque;
	torques[swingHipIndex]:= swingHipTorque;
end;

procedure TSimBiController.blendOutTorques;
const
	hMin = 0.2;
	hMax = 0.4;
var
  h: Double;
  i: Integer;
begin
	h:= character.Root.CMPosition.y;
  h:= clamp(h, hMin, hMax);

	h:= (h-hMin) / (hMax-hMin);

	for i:=0 to Length(torques) - 1 do
		torques[i]:= torques[i]*h + 0 * (1-h);
end;

procedure TSimBiController.resolveJoints(state: TSimBiConState);
var
  i: Integer;
  jt: TTrajectory;
  tmpName: String;
begin
	for i:=0 to state.sTraj.Count - 1 do
  begin
		jt:= state.sTraj[i];
		//deal with the 'root' special case
		if jt.jName = 'root' then
    begin
			jt.leftStanceIndex:= -1;
      jt.rightStanceIndex:= -1;
			continue;
		end;
		//deal with the SWING_XXX' case
		if jt.jName.StartsWith('SWING_') then
    begin
      tmpName:= ' ' + Copy(jt.jName, 1 + Length('SWING_'));
			tmpName[1]:= 'r';
			jt.leftStanceIndex:= character.getJointIndex(tmpName);
			if jt.leftStanceIndex < 0 then
				raise Exception.CreateFmt('Cannot find joint %s', [tmpName]);
			tmpName[1]:= 'l';
			jt.rightStanceIndex:= character.getJointIndex(tmpName);
			if jt.rightStanceIndex < 0 then
				raise Exception.CreateFmt('Cannot find joint %s', [tmpName]);
			continue;
		end;
		//deal with the STANCE_XXX' case
		if jt.jName.StartsWith('STANCE_') then
    begin
      tmpName:= ' ' + Copy(jt.jName, 1 + Length('STANCE_'));
			tmpName[1]:= 'l';
			jt.leftStanceIndex:= character.getJointIndex(tmpName);
			if jt.leftStanceIndex < 0 then
				raise Exception.CreateFmt('Cannot find joint %s', [tmpName]);
			tmpName[1]:= 'r';
			jt.rightStanceIndex:= character.getJointIndex(tmpName);
			if jt.rightStanceIndex < 0 then
				raise Exception.CreateFmt('Cannot find joint %s', [tmpName]);
			continue;
		end;
		//if we get here, it means it is just the name...
		jt.leftStanceIndex:= character.getJointIndex(jt.jName);
		if jt.leftStanceIndex < 0 then
				raise Exception.CreateFmt('Cannot find joint %s', [jt.jName]);
		jt.rightStanceIndex:= jt.leftStanceIndex;
	end;
end;

procedure TSimBiController.setStance(newStance: Integer);
begin
	stance:= newStance;
	if stance = LEFT_STANCE then
  begin
		stanceFoot:= lFoot;
		swingFoot:= rFoot;
		stanceHipIndex:= lHipIndex;
		swingHipIndex:= rHipIndex;
	end
  else
  begin
		stanceFoot:= rFoot;
		swingFoot:= lFoot;
		stanceHipIndex:= rHipIndex;
		swingHipIndex:= lHipIndex;
	end;
end;

function TSimBiController.getState(idx: Integer): TSimBiConState;
begin
	if idx >= states.Count then Exit(nil);
	Exit(states[idx]);
end;

function TSimBiController.getStateCount: Integer;
begin
  Exit(states.Count);
end;

procedure TSimBiController.addState(state_disown: TSimBiConState);
begin
  states.Add(state_disown);
  resolveJoints(state_disown);
end;

procedure TSimBiController.clearStates;
begin
  states.Count:= 0;
end;

procedure TSimBiController.computeTorques(cfs: TList<TContactPoint>);
begin
	if FSMStateIndex < 0 then
		setFSMStateTo(startingState);

	if FSMStateIndex >= states.Count then
  begin
		// WriteLn('Warning: no FSM state was selected in the controller!');
		Exit;
	end;

	evaluateJointTargets();
	computePDTorques(cfs);
	//and now separetely compute the torques for the hips - together with the feedback term, this is what defines simbicon
	computeHipTorques(aRootD, getStanceFootWeightRatio(cfs), 0);
  blendOutTorques();
end;

procedure TSimBiController.evaluateJointTargets;
var
  poseRS: TCharacterState;
  i: Integer;
  phiToUse: Double;
  curState: TSimBiConState;
  newOrientation: Double;
  jIndex: Integer;
  d0, v0: TVector2;
begin
  poseRS:= desiredPose;

	updateDAndV();

	//always start from a neutral desired pose, and build from there...
	for i:=0 to jointCount - 1 do
  begin
		if not controlParams[i].qRelExternallyComputed then
    begin
			poseRS.setJointRelativeOrientation(0, i);
			poseRS.setJointRelativeAngVelocity(0, i);
		end;
		controlParams[i].controlled:= true;
		controlParams[i].relToFrame:= false;
	end;

	aRootD:= 0;

	phiToUse:= Min(phi, 1);

	curState:= states[FSMStateIndex];

	for i:=0 to curState.getTrajectoryCount - 1 do
  begin
		jIndex:= curState.sTraj[i].getJointIndex(stance);
		if (jIndex > -1) and controlParams[jIndex].qRelExternallyComputed then
			continue;

		computeD0(phiToUse, d0);
		computeV0(phiToUse, v0);
		newOrientation:= curState.sTraj[i].evaluateTrajectory(Self, character.getJoint(jIndex), stance, phiToUse, d - d0, v - v0);

		if jIndex = -1 then
    begin
			aRootD:= newOrientation;
			rootControlParams.strength:= curState.sTraj[i].evaluateStrength(phiToUse);
		end
    else
    begin
			if curState.sTraj[i].relToCharFrame or (jIndex = swingHipIndex) then
				controlParams[jIndex].relToFrame:= true;
			poseRS.setJointRelativeOrientation(newOrientation, jIndex);
			controlParams[jIndex].strength:= curState.sTraj[i].evaluateStrength(phiToUse);
		end;
	end;
end;

procedure TSimBiController.computePDTorques(cfs: TList<TContactPoint>);
begin
  inherited computeTorques(cfs);
end;

function TSimBiController.performPostTasks(dt: Double; cfs: TList<TContactPoint>): Boolean;
var
  transition: Boolean;
begin
  inherited;
  transition:= advanceInTime(dt, cfs) <> -1;
  updateDAndV;
  Result:= transition;
end;

function TSimBiController.advanceInTime(dt: Double; cfs: TList<TContactPoint>): Integer;
var
  i, newStateIndex: Integer;
begin
	if FSMStateIndex < 0 then
		setFSMStateTo(startingState);

	if dt <= 0 then
		Exit(-1);

	if FSMStateIndex >= states.Count then
  begin
		// WriteLn('Warning: no FSM state was selected in the controller!');
		Exit(-1);
	end;

	bodyTouchedTheGround:= false;
	//see if anything else other than the feet touch the ground...
	for i:=0 to cfs.Count - 1 do
  begin
		if isFoot(cfs[i].rb1) or isFoot(cfs[i].rb2) then
			continue;

		bodyTouchedTheGround:= true;
		break;
	end;

	//advance the phase of the controller
	Self.phi:= Self.phi + dt/states[FSMStateIndex].StateTime;

	//see if we have to transition to the next state in the FSM, and do it if so...
	if states[FSMStateIndex].needTransition(phi, Abs(b2Dot(getForceOnFoot(swingFoot, cfs), Up)), Abs(b2Dot(getForceOnFoot(stanceFoot, cfs), Up))) then
  begin
		newStateIndex:= states[FSMStateIndex].NextStateIndex;
		transitionToState(newStateIndex);
		Exit(newStateIndex);
	end;

	//if we didn't transition to a new state...
	Exit(-1);
end;

function TSimBiController.getStanceFootPos: TVector2;
begin
  if Assigned(stanceFoot) then
    Exit(stanceFoot.CMPosition);
  Result:= TVector2.From(0, 0);
end;

function TSimBiController.getSwingFootPos: TVector2;
begin
  if Assigned(swingFoot) then
    Exit(swingFoot.CMPosition);
  Result:= TVector2.From(0, 0);
end;

function TSimBiController.getFSMState: Integer;
begin
  if FSMStateIndex < 0 then
    setFSMStateTo(startingState);
  Result:= Self.FSMStateIndex;
end;

procedure TSimBiController.updateDAndV;
begin
	comPosition:= character.getCOM();

	comVelocity:= character.getCOMVelocity();

	d:= VectorDelta(stanceFoot.CMPosition, comPosition);
	v:= comVelocity;

	//and now compute the vector from the COM to the center of midpoint between the feet, again expressed in world coordinates
	feetMidpoint:= (stanceFoot.CMPosition + swingFoot.CMPosition) / 2;
  feetMidpoint:= swingFoot.CMPosition;

	//now we have to compute the difference between the current COM and the desired COMPosition, in world coordinates
	doubleStanceCOMError:= VectorDelta(comPosition, feetMidpoint);
	//and add the user specified offset
	doubleStanceCOMError.y:= 0;
end;

procedure TSimBiController.computeD0(phi: Double; var d0: TVector2);
var
  currState: TSimBiConState;
begin
  currState:= states[getFSMState()];
  computeDorV(phi, currState.dTrajX, stance, d0);
end;

procedure TSimBiController.computeV0(phi: Double; var v0: TVector2);
var
  currState: TSimBiConState;
begin
  currState:= states[getFSMState()];
  computeDorV(phi, currState.vTrajX, stance, v0);
end;

class procedure TSimBiController.computeDorV(phi: Double; trajX: TTrajectory1d; stance: Integer; var result: TVector2);
begin
  result.y:= 0;
  if trajX = nil then
    result.x:= 0
  else
    result.x:= trajX.evaluate_catmull_rom(phi);
end;

procedure TSimBiController.addControlParams(const JointName: String; kp, kd, tauMax, scale: Double);
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
  addControlParams(params);
end;

function TSimBiController.addTrajectory(state: TSimBiConState; const JointName: String; const BaseTrajectoryArray: array of Double;
  relToCharFrame: Boolean; reverseOnStance: Integer) : TTrajectory;
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

  Result:= traj;
end;

procedure TSimBiController.addStrengthTrajectory(traj: TTrajectory; const StrengthTrajectoryArray: array of Double);
var
  strengthTraj: TTrajectory1d;
  i: Integer;
begin
  strengthTraj:= TTrajectory1d.Create;
  i:= 0;
  while i < Length(StrengthTrajectoryArray) do
  begin
    strengthTraj.addKnot(StrengthTrajectoryArray[i], StrengthTrajectoryArray[i+1]);
    Inc(i, 2);
  end;
  traj.setStrengthTrajectory(strengthTraj);
end;

procedure TSimBiController.addLinearBalanceFeedback(traj: TTrajectory; cd, cv: Double);
var
  feedback: TLinearBalanceFeedback;
begin
  feedback:= TLinearBalanceFeedback.Create;
  feedback.cd:= cd;
  feedback.cv:= cv;
  traj.components[0].bFeedback:= feedback;
end;

{ TTrajectoryComponent }

procedure TTrajectoryComponent.setReverseOnStance(stance: Integer);
begin
  if stance = ROS_LEFT then
  begin
    reverseAngleOnLeftStance:= true;
    reverseAngleOnRightStance:= false;
  end
  else
  if stance = ROS_RIGHT then
  begin
    reverseAngleOnLeftStance:= false;
    reverseAngleOnRightStance:= true;
  end
  else
  if stance = ROS_DONT_REVERSE then
  begin
    reverseAngleOnLeftStance:= false;
    reverseAngleOnRightStance:= false;
  end
  else
    raise Exception.Create('Invalid state stance!');
end;

function TTrajectoryComponent.getReverseOnStance: Integer;
begin
  if reverseAngleOnLeftStance and reverseAngleOnRightStance then
    raise Exception.Create('Invalid state stance!');
  if reverseAngleOnLeftStance then
    Exit(ROS_LEFT);
  if reverseAngleOnRightStance then
    Exit(ROS_RIGHT);

  Exit(ROS_DONT_REVERSE);
end;

procedure TTrajectoryComponent.setBaseTrajectory(traj: TTrajectory1d);
begin
  baseTraj:= traj;
end;

function TTrajectoryComponent.getBaseTrajectory: TTrajectory1d;
begin
  Exit(baseTraj);
end;

procedure TTrajectoryComponent.setVTrajScale(traj: TTrajectory1d);
begin
  vTrajScale:= traj;
end;

function TTrajectoryComponent.getVTrajScale: TTrajectory1d;
begin
  Exit(vTrajScale);
end;

procedure TTrajectoryComponent.setDTrajScale(traj: TTrajectory1d);
begin
  dTrajScale:= traj;
end;

function TTrajectoryComponent.getDTrajScale: TTrajectory1d;
begin
  Exit(dTrajScale);
end;

function TTrajectoryComponent.evaluateTrajectoryComponent(con: TSimBiController; j: TJoint; stance: Integer; phi: Double; const d,
  v: TVector2; bareTrajectory: Boolean): Double;
var
  baseAngle, scale, feedbackValue: Double;
begin
  baseAngle:= offset;

  scale:= 1;
  //this d.z should really be d dotted with some axis - probably the same as the feedback one...
  // не используется
  if Assigned(dTrajScale) then
    scale:= scale * dTrajScale.evaluate_linear(d.x);

  //this v.z should really be v dotted with some axis - probably the same as the feedback one...
  // не используется
  if Assigned(vTrajScale) then
    scale:= scale * vTrajScale.evaluate_linear(v.x);

  // не используется
  if bareTrajectory then
    scale:= 1;

  if Assigned(baseTraj) then
    baseAngle:= baseAngle + baseTraj.evaluate_catmull_rom(phi) * scale;

  if (stance = LEFT_STANCE) and reverseAngleOnLeftStance then
    baseAngle:= -baseAngle;
  if (stance = RIGHT_STANCE) and reverseAngleOnRightStance then
    baseAngle:= -baseAngle;

  feedbackValue:= computeFeedback(con, j, phi, d, v);

  Result:= baseAngle + feedbackValue;
end;

function TTrajectoryComponent.computeFeedback(con: TSimBiController; j: TJoint; phi: double; const d, v: TVector2): Double;
begin
  if bFeedback = nil then
    Exit(0);
  Result:= bFeedback.getFeedbackContribution(con, j, phi, d, v);
end;

{ TTrajectory }

constructor TTrajectory.Create;
begin
  components:= TList<TTrajectoryComponent>.Create;
  leftStanceIndex:= -1;
  rightStanceIndex:= -1;
  jName:= 'NoNameJoint';
end;

procedure TTrajectory.setStrengthTrajectory(traj: TTrajectory1d);
begin
  strengthTraj:= traj;
end;

function TTrajectory.getStrengthTrajectory: TTrajectory1d;
begin
  Exit(strengthTraj);
end;

procedure TTrajectory.setRelativeToCharacterFrame(rel: Boolean);
begin
  relToCharFrame:= rel;
end;

function TTrajectory.isRelativeToCharacterFrame: Boolean;
begin
  Exit(relToCharFrame);
end;

procedure TTrajectory.addTrajectoryComponent(trajComp_disown: TTrajectoryComponent);
begin
  components.Add(trajComp_disown);
end;

procedure TTrajectory.clearTrajectoryComponents;
begin
  // todo free objects
  components.Count:= 0;
end;

function TTrajectory.getTrajectoryComponent(index: Integer): TTrajectoryComponent;
begin
  Exit(components[index]);
end;

function TTrajectory.getTrajectoryComponentCount: Integer;
begin
  Exit(components.Count);
end;

function TTrajectory.evaluateTrajectory(con: TSimBiController; j: TJoint; stance: Integer; phi: Double; const d, v: TVector2;
  bareTrajectory: Boolean): Double;
var
  a: Double;
  i: Integer;
begin
  a:= 0;

  for i:=0 to components.Count - 1 do
    a:= components[i].evaluateTrajectoryComponent(con, j, stance, phi, d, v, bareTrajectory) + a;

  Exit(a);
end;

function TTrajectory.evaluateStrength(phiToUse: Double): Double;
begin
  if strengthTraj = nil then Exit(1.0);
  Exit(strengthTraj.evaluate_catmull_rom(phiToUse));
end;

function TTrajectory.getJointIndex(stance: Integer): Integer;
begin
  if stance = LEFT_STANCE then Result:= leftStanceIndex
  else Result:= rightStanceIndex;
end;

{ TSimBiConState }

constructor TSimBiConState.Create;
begin
  sTraj:= TList<TTrajectory>.Create;

  Name:= 'Uninitialized state';
  nextStateIndex:= -1;
  Self.stateTime:= 0;
  transitionOnFootContact:= true;
  minPhiBeforeTransitionOnFootContact:= 0.5;
  minSwingFootForceForContact:= 20.0;
  reverseStance:= false;
  keepStance:= false;

  dTrajX:= nil;
  vTrajX:= nil;
end;

function TSimBiConState.getStateStance(oldStance: Integer): Integer;
begin
  if keepStance then
    Exit(oldStance);
  if not reverseStance then
    Exit(stateStance);
  if oldStance = LEFT_STANCE then
    Result:= RIGHT_STANCE
  else Result:= LEFT_STANCE;
end;

function TSimBiConState.getTrajectoryCount: Integer;
begin
  Exit(sTraj.Count);
end;

function TSimBiConState.getTrajectory(idx: Integer): TTrajectory;
begin
  if idx >= sTraj.Count then Exit(nil);
  Exit(sTraj[idx]);
end;

function TSimBiConState.getTrajectory(const name: String): TTrajectory;
var
  i: Integer;
begin
  for i:=0 to sTraj.Count - 1 do
    if sTraj[i].jName = name then
      Exit(sTraj[i]);
  Result:= nil;
end;

function TSimBiConState.needTransition(phi, swingFootVerticalForce, stanceFootVerticalForce: Double): Boolean;
begin
  //if it is a foot contact based transition
  if transitionOnFootContact then
  begin
    //transition if we have a meaningful foot contact, and if it does not happen too early on...
    if ((phi > minPhiBeforeTransitionOnFootContact) and (swingFootVerticalForce > minSwingFootForceForContact)) or (phi >= 1) then
      Exit(true);
    Exit(false);
  end;

  //otherwise it must be a time-based transition
  if phi >= 1 then
    Exit(true);

  Exit(false);
end;

procedure TSimBiConState.setNextStateIndex(nextStateIndex: Integer);
begin
  Self.nextStateIndex:= nextStateIndex;
end;

procedure TSimBiConState.setTransitionOnFootContact(transition: Boolean);
begin
  Self.transitionOnFootContact:= transition;
end;

function TSimBiConState.getTransitionOnFootContact: Boolean;
begin
  Exit(Self.transitionOnFootContact);
end;

procedure TSimBiConState.setStance(stanceType: Integer);
begin
  if stanceType = STATE_LEFT_STANCE then
  begin
    reverseStance:= false;
    keepStance:= false;
    stateStance:= LEFT_STANCE;
  end
  else
  if stanceType = STATE_RIGHT_STANCE then
  begin
    reverseStance:= false;
    keepStance:= false;
    stateStance:= RIGHT_STANCE;
  end
  else
  if stanceType = STATE_REVERSE_STANCE then
  begin
    reverseStance:= true;
    keepStance:= false;
    stateStance:= -1;
  end
  else
  if stanceType = STATE_KEEP_STANCE then
  begin
    reverseStance:= false;
    keepStance:= true;
    stateStance:= -1;
  end
  else
    raise Exception.Create('Invalid state stance!');
end;

function TSimBiConState.getStance: Integer;
begin
  if reverseStance and keepStance then
    raise Exception.Create('Invalid state stance!');
  if reverseStance then
    Exit(STATE_REVERSE_STANCE);
  if keepStance then
    Exit(STATE_KEEP_STANCE);

  if stateStance = LEFT_STANCE then
    Result:= STATE_LEFT_STANCE
  else Result:= STATE_RIGHT_STANCE;
end;

procedure TSimBiConState.addTrajectory(traj_disown: TTrajectory);
begin
  sTraj.Add(traj_disown);
end;

procedure TSimBiConState.clearTrajectories;
begin
  sTraj.Count:= 0;
end;

{ TVirtualModelController }

procedure TVirtualModelController.computeTorques(cfs: TList<TContactPoint>);
begin
  raise ENotSupportedException.Create('don''t call this method...');
end;

procedure TVirtualModelController.computeJointTorquesEquivalentToForce(start: TJoint; const pLocal, fGlobal: TVector2; _end: TJoint);
var
  currentJoint: TJoint;
  pGlobal, tmpV: TVector2;
  tmpT: Double;
begin
	currentJoint:= start;
	pGlobal:= start.child.getWorldCoordinatesPoint(pLocal);

	while currentJoint <> _end do
  begin
		if currentJoint = nil then
			raise Exception.Create('VirtualModelController::computeJointTorquesEquivalentToForce --> end was not a parent of start...');
		tmpV:= VectorDelta(currentJoint.parent.getWorldCoordinatesPoint(currentJoint.pJPos), pGlobal);
		tmpT:= b2Cross(tmpV, fGlobal);
		torques[currentJoint.id]:= torques[currentJoint.id] - tmpT;
		currentJoint:= currentJoint.parent.parentJoint;
	end;

	//and we just have to do it once more for the end joint, if it's not NULL
	if _end <> nil then
  begin
		tmpV:= VectorDelta(currentJoint.parent.getWorldCoordinatesPoint(currentJoint.pJPos), pGlobal);
		torques[currentJoint.id]:= torques[currentJoint.id] - b2Cross(tmpV, fGlobal);
	end;
end;

{ TIKVMCController }

constructor TIKVMCController.Create(b: TCharacter; const Name: String);
begin
  inherited;
  swingFootTrajectorySagittal:= TTrajectory1d.Create;
  swingFootHeightTrajectory:= TTrajectory1d.Create;

	lKneeIndex:= character.getJointIndex('lKnee');
	rKneeIndex:= character.getJointIndex('rKnee');
	lAnkleIndex:= character.getJointIndex('lAnkle');
	rAnkleIndex:= character.getJointIndex('rAnkle');

	vmc:= TVirtualModelController.Create(b, 'VMC');
end;

procedure TIKVMCController.setBehaviour(behaviour_disown: TBehaviourController);
begin
  if Self.behaviour <> nil then
    raise Exception.Create('Behaviour already set!');
  Self.behaviour:= behaviour_disown;  
end;

function TIKVMCController.computeIPStepLocation : TVector2;
var
  step: TVector2;
  h: Double;
begin
	h:= Abs(comPosition.y - stanceFoot.CMPosition.y);
	step.x:= v.x * sqrt(h/-Gravity + v.x * v.x / (4*-Gravity*-Gravity)) * 1.1;
	step.y:= 0;
	Result:= step;
end;

procedure TIKVMCController.computeIKSwingLegTargets(dt: Double);
var
  pNow, pFuture, parentAxis, childAxis: TVector2;
begin
	pNow:= getSwingFootTargetLocation(phi, comPosition);
	pFuture:= getSwingFootTargetLocation(Min(phi+dt, 1), comPosition + comVelocity * dt);

	parentAxis:= VectorDelta(character.joints[swingHipIndex].cJPos, character.joints[swingKneeIndex].pJPos);
	childAxis:= VectorDelta(character.joints[swingKneeIndex].cJPos, character.joints[swingAnkleIndex].pJPos);

	computeIKQandW(swingHipIndex, swingKneeIndex, parentAxis, 1, 1, childAxis, pNow, true, pFuture, dt);
end;

procedure TIKVMCController.computeIKArmsTargets(dt: Double);
var
  pNow, pFuture, parentAxis, childAxis: TVector2;
begin
	pNow:= armsTarget;
	pFuture:= pNow + comVelocity * dt;

	parentAxis:= VectorDelta(character.joints[rShoulderIndex].cJPos, character.joints[rElbowIndex].pJPos);
	childAxis:= VectorDelta(character.joints[rElbowIndex].cJPos, -character.joints[rElbowIndex].cJPos);
	computeIKQandW(rShoulderIndex, rElbowIndex, parentAxis, -1, -1, childAxis, pNow, true, pFuture, dt);
  {
	parentAxis:= VectorDelta(character.joints[lShoulderIndex].cJPos, character.joints[lElbowIndex].pJPos);
	childAxis:= VectorDelta(character.joints[lElbowIndex].cJPos, -character.joints[lElbowIndex].cJPos);
	computeIKQandW(lShoulderIndex, lElbowIndex, parentAxis, -1, -1, childAxis, pNow, true, pFuture, dt);
  }
end;


procedure TIKVMCController.DebugComputeIKSwingLegTargets(const Point: TVector2);
var
  pNow, pFuture, parentAxis, childAxis: TVector2;
begin
	pNow:= Point;
	pFuture:= Point;

	parentAxis:= VectorDelta(character.joints[swingHipIndex].cJPos, character.joints[swingKneeIndex].pJPos);
	childAxis:= VectorDelta(character.joints[swingKneeIndex].cJPos, character.joints[swingAnkleIndex].pJPos);

	computeIKQandW(swingHipIndex, swingKneeIndex, parentAxis, 1, 1, childAxis, pNow, False, pFuture, 0.001);
end;

procedure TIKVMCController.computeTorques(cfs: TList<TContactPoint>);
begin
	evaluateJointTargets();

	//now overwrite the target angles for the swing hip and the swing knee in order to ensure foot-placement control
	if not doubleStanceMode then
  begin
		computeIKSwingLegTargets(0.001);
    // computeIKArmsTargets(0.001);
  end;

	computePDTorques(cfs);

	bubbleUpTorques();

	computeGravityCompensationTorques();

	ffRootTorque:= 0;

	if cfs.Count > 0 then
		COMJT(cfs);

	if doubleStanceMode then
  begin
    if cfs.Count > 0 then
  		computeLegTorques(swingAnkleIndex, swingKneeIndex, swingHipIndex, cfs);
  end
  else
  	computeHipTorques(aRootD, getStanceFootWeightRatio(cfs), ffRootTorque);

  blendOutTorques;
end;

function TIKVMCController.computeSwingFootDelta(phiToUse: Double; stanceToUse: Integer): TVector2;
begin
  if phiToUse < 0 then
    phiToUse:= phi;
  if phiToUse > 1 then
    phiToUse:= 1;
  if stanceToUse < 0 then
    stanceToUse:= stance;
  if stanceToUse > 1 then
    stanceToUse:= 1;

  Result:= TVector2.From(swingFootTrajectoryDeltaSagittal.evaluate_catmull_rom(phiToUse),
    swingFootTrajectoryDeltaHeight.evaluate_catmull_rom(phiToUse));
end;

function TIKVMCController.getSwingFootTargetLocation(t: Double; const com: TVector2): TVector2;
var
  step: TVector2;
begin
	step.x:= swingFootTrajectorySagittal.evaluate_catmull_rom(t);
  step.y:= 0;
	//add it to the com location
	step:= com + step;
	//finally, set the desired height of the foot
	step.y:= swingFootHeightTrajectory.evaluate_catmull_rom(t) + panicHeight + unplannedForHeight;

  step:= step + computeSwingFootDelta(t);

	Result:= step;
end;

procedure TIKVMCController.computeGravityCompensationTorques;
var
  i: Integer;
begin
  vmc.resetTorques;
	for i:=0 to character.joints.Count - 1 do
  begin
		if (i <> stanceHipIndex) and (i <> stanceKneeIndex) and (i <> stanceAnkleIndex) then
			vmc.computeJointTorquesEquivalentToForce(character.joints[i], TVector2.From(0, 0), TVector2.From(0, character.joints[i].child.GetMass*-Gravity), nil);
	end;

	for i:=0 to character.joints.Count - 1 do
		torques[i]:= torques[i] + vmc.torques[i];
end;

procedure TIKVMCController.updateSwingAndStanceReferences;
begin
	stanceHipIndex:= IfThen(stance = LEFT_STANCE, lHipIndex, rHipIndex);
	swingHipIndex:= IfThen(stance = LEFT_STANCE, rHipIndex, lHipIndex);
	stanceKneeIndex:= IfThen(stance = LEFT_STANCE, lKneeIndex, rKneeIndex);
	swingKneeIndex:= IfThen(stance = LEFT_STANCE, rKneeIndex, lKneeIndex);
	stanceAnkleIndex:= IfThen(stance = LEFT_STANCE, lAnkleIndex, rAnkleIndex);
	swingAnkleIndex:= IfThen(stance = LEFT_STANCE, rAnkleIndex, lAnkleIndex);

	swingHip:= character.joints[swingHipIndex];
	swingKnee:= character.joints[swingKneeIndex];
end;

procedure boundToRange(var v: Double; min, max: Double); inline;
begin
	if v < min then
		v:= min
  else
	if v > max then
		v:= max;
end;

procedure TIKVMCController.computeIKQandW(parentJIndex, childJIndex: Integer; const parentAxis: TVector2; parentNormal, childNormal: Double;
  const childEndEffector, wP: TVector2; computeAngVelocities: Boolean; const futureWP: TVector2; dt: Double);
var
  rs: TCharacterState;
  parentJoint: TJoint;
  gParent: TArticulatedRigidBody; 
  aParent, aChild,
  wParentD, wChildD: Double;
  velOffset: TVector2;
  aParentF, aChildF, 
  aDiff, aDiffv: Double;
begin
	rs:= desiredPose; 

	parentJoint:= character.joints[parentJIndex]; 
	gParent:= parentJoint.parent;

	TwoLinkIK.getIKOrientations(parentJoint.ParentJointPosition, gParent.getLocalCoordinatesPoint(wP), parentNormal, parentAxis,
    childNormal, childEndEffector, aParent, aChild);

	controlParams[parentJIndex].relToFrame:= false;
	controlParams[childJIndex].relToFrame:= false;
	rs.setJointRelativeOrientation(aParent, parentJIndex);
	rs.setJointRelativeOrientation(aChild, childJIndex);

	wParentD:= 0;
	wChildD:= 0;

	if computeAngVelocities then
  begin
		//the joint's origin will also move, so take that into account, by subbing the offset by which it moves to the
		//futureTarget (to get the same relative position to the hip)
		velOffset:= gParent.getAbsoluteVelocityForLocalPoint(parentJoint.ParentJointPosition);

		TwoLinkIK.getIKOrientations(parentJoint.ParentJointPosition, gParent.getLocalCoordinatesPoint(futureWP + velOffset * -dt),
      parentNormal, parentAxis, childNormal, childEndEffector, aParentF, aChildF);

		aDiff:= aParentF + -aParent;
    aDiffv:= Sin(aDiff/2);
		wParentD:= aDiffv * 2/dt;
		//the above is the desired angular velocity, were the parent not rotating already - but it usually is, so we'll account for that
		wParentD:= wParentD - gParent.GetAngularVelocity;

		aDiff:= aChildF + -aChild;
    aDiffv:= Sin(aDiff/2);
		wChildD:= aDiffv * 2/dt;

		//make sure we don't go overboard with the estimates, in case there are discontinuities in the trajectories...
		boundToRange(wChildD, -5, 5);
		boundToRange(wParentD, -5, 5);
	end;

	rs.setJointRelativeAngVelocity(wParentD, parentJIndex);
	rs.setJointRelativeAngVelocity(wChildD, childJIndex);
end;

procedure TIKVMCController.bubbleUpTorques;
var
  i: Integer;
begin
	for i:=character.joints.Count - 1 downto 0 do
  begin
		if (i <> stanceHipIndex) and (i <> stanceKneeIndex) then
			if character.joints[i].Parent <> root then
				torques[character.joints[i].Parent.ParentJoint.id]:= torques[character.joints[i].Parent.ParentJoint.id] + torques[i];
	end;
end;

procedure TIKVMCController.computeLegTorques(ankleIndex, kneeIndex, hipIndex: Integer; cfs: TList<TContactPoint>);
var
  fA, p, r: TVector2;
  ankleTorque: Double;
  lBackIndex, mBackIndex: Integer;
begin
	fA:= computeVirtualForce();

	p:= comPosition;

	r:= VectorDelta(character.joints[ankleIndex].child.getWorldCoordinatesPoint(character.joints[ankleIndex].cJPos), p);

	ankleTorque:= b2Cross(r, fA);
	preprocessAnkleVTorque(ankleIndex, cfs, ankleTorque);
	torques[ankleIndex]:= torques[ankleIndex] + ankleTorque;

	r:= VectorDelta(character.joints[kneeIndex].child.getWorldCoordinatesPoint(character.joints[kneeIndex].cJPos), p);
	torques[kneeIndex]:= torques[kneeIndex] + b2Cross(r, fA);

	r:= VectorDelta(character.joints[hipIndex].child.getWorldCoordinatesPoint(character.joints[hipIndex].cJPos), p);
	torques[hipIndex]:= torques[hipIndex] + b2Cross(r, fA);

	//the torque on the stance hip is cancelled out, so pass it in as a torque that the root wants to see!
	ffRootTorque:= ffRootTorque - b2Cross(r, fA);

	lBackIndex:= character.getJointIndex('pelvis_lowerback');
	r:= VectorDelta(character.joints[lBackIndex].child.getWorldCoordinatesPoint(character.joints[lBackIndex].cJPos), p);
	torques[lBackIndex]:= torques[lBackIndex] + b2Cross(r, fA) / 10;

	mBackIndex:= character.getJointIndex('lowerback_torso');
	r:= VectorDelta(character.joints[mBackIndex].child.getWorldCoordinatesPoint(character.joints[mBackIndex].cJPos), p);
	torques[mBackIndex]:= torques[mBackIndex] + b2Cross(r, fA) / 10;
end;

procedure TIKVMCController.COMJT(cfs: TList<TContactPoint>);
var
  lBackIndex, mBackIndex: Integer;
  m: Double;
  tibia, femur, pelvis, lBack, mBack: TArticulatedRigidBody;
  anklePos, kneePos, hipPos, lbackPos, mbackPos,
  fA, f1, f2, f3, f4, f5: TVector2;
  ankleTorque: Double;
begin
	lBackIndex:= character.getJointIndex('pelvis_lowerback');
	mBackIndex:= character.getJointIndex('lowerback_torso');

	tibia:= character.joints[stanceAnkleIndex].parent;
	femur:= character.joints[stanceKneeIndex].parent;
	pelvis:= character.joints[stanceHipIndex].parent;
	lBack:= character.joints[lBackIndex].child;
	mBack:= character.joints[mBackIndex].child;

	anklePos:= character.joints[stanceAnkleIndex].child.getWorldCoordinatesPoint(character.joints[stanceAnkleIndex].cJPos);
	kneePos:= character.joints[stanceKneeIndex].child.getWorldCoordinatesPoint(character.joints[stanceKneeIndex].cJPos);
	hipPos:= character.joints[stanceHipIndex].child.getWorldCoordinatesPoint(character.joints[stanceHipIndex].cJPos);
	lbackPos:= character.joints[lBackIndex].child.getWorldCoordinatesPoint(character.joints[lBackIndex].cJPos);
	mbackPos:= character.joints[mBackIndex].child.getWorldCoordinatesPoint(character.joints[mBackIndex].cJPos);

	//total mass...
	m:= tibia.GetMass + femur.GetMass + pelvis.GetMass + lBack.GetMass + mBack.GetMass;

	fA:= computeVirtualForce();

	f1:=	VectorDelta(anklePos, tibia.GetPosition) * tibia.GetMass +
					VectorDelta(anklePos, femur.GetPosition) * femur.GetMass +
					VectorDelta(anklePos, pelvis.GetPosition) * pelvis.GetMass +
					VectorDelta(anklePos, lBack.GetPosition) * lBack.GetMass +
					VectorDelta(anklePos, mBack.GetPosition) * mBack.GetMass;
	f1:= f1 / m;

	f2:=	VectorDelta(kneePos, femur.GetPosition) * femur.GetMass +
					VectorDelta(kneePos, pelvis.GetPosition) * pelvis.GetMass +
					VectorDelta(kneePos, lBack.GetPosition) * lBack.GetMass +
					VectorDelta(kneePos, mBack.GetPosition) * mBack.GetMass;
	f2:= f2 / m;

	f3:=	VectorDelta(hipPos, pelvis.GetPosition) * pelvis.GetMass +
					VectorDelta(hipPos, lBack.GetPosition) * lBack.GetMass +
					VectorDelta(hipPos, mBack.GetPosition) * mBack.GetMass;
	f3:= f3 / m;

	f4:=	VectorDelta(lbackPos, lBack.GetPosition) * lBack.GetMass +
					VectorDelta(lbackPos, mBack.GetPosition) * mBack.GetMass;
	f4:= f4 / m;

	f5:=	VectorDelta(mbackPos, mBack.GetPosition) * mBack.GetMass;
	f5:= f5 / m;

	ankleTorque:= b2Cross(f1, fA);
	preprocessAnkleVTorque(stanceAnkleIndex, cfs, ankleTorque);

	torques[stanceAnkleIndex]:= torques[stanceAnkleIndex] + ankleTorque;
	torques[stanceKneeIndex]:= torques[stanceKneeIndex] + b2Cross(f2, fA);
	torques[stanceHipIndex]:= torques[stanceHipIndex] + b2Cross(f3, fA);

	//the torque on the stance hip is cancelled out, so pass it in as a torque that the root wants to see!
	ffRootTorque:= ffRootTorque - b2Cross(f3, fA);

	torques[lBackIndex]:= torques[lBackIndex] - b2Cross(f4, fA) * 0.5;
	torques[mBackIndex]:= torques[mBackIndex] - b2Cross(f5, fA) * 0.3;
end;

function TIKVMCController.computeVirtualForce: TVector2;
var
  desA, fA: TVector2;
begin
	//this is the desired acceleration of the center of mass
	desA.x:= (velDSagittal - v.x) * 30;
	desA.y:= 0;

	if doubleStanceMode then
  begin
		desA.x:= (doubleStanceCOMError.x + comOffsetSagittal) * 10 + (velDSagittal - v.x) * 150;
	end;

	//and this is the force that would achieve that - make sure it's not too large...
	fA:= desA * character.Mass;
	boundToRange(fA.x, -60, 60);

	Result:= fA;
end;

procedure TIKVMCController.preprocessAnkleVTorque(ankleJointIndex: Integer; cfs: TList<TContactPoint>; var ankleVTorque: Double);
var
  heelInContact, toeInContact: Boolean;
  foot: TArticulatedRigidBody;
  footRelativeAngularVel: Double;
begin
	foot:= character.joints[ankleJointIndex].child;
	getForceInfoOn(foot, cfs, heelInContact, toeInContact);

	if not toeInContact or (phi < 0.2) or (phi > 0.8) then ankleVTorque:= 0;

	footRelativeAngularVel:= foot.GetAngularVelocity;

	if Abs(footRelativeAngularVel) > 1.0 then ankleVTorque:= 0;
end;

procedure TIKVMCController.performPreTasks(dt: Double; cfs: TList<TContactPoint>);
begin
	if Assigned(behaviour) then
		behaviour.simStepPlan(dt);

	inherited performPreTasks(dt, cfs);
end;

function TIKVMCController.performPostTasks(dt: Double; cfs: TList<TContactPoint>): Boolean;
var
  newState: Boolean;
begin
	newState:= inherited performPostTasks(dt, cfs);

	if newState then
		if Assigned(behaviour) then
			behaviour.conTransitionPlan();

	Exit(newState);
end;

procedure TIKVMCController.getForceInfoOn(rb: TArticulatedRigidBody; cfs: TList<TContactPoint>; var heelForce, toeForce: Boolean);
var
  i: Integer;
  tmpP: TVector2;
begin
	//figure out if the toe/heel are in contact...
	heelForce:= false;
	toeForce:= false;
	for i:=0 to cfs.Count - 1 do
  begin
		if haveRelationBetween(cfs[i].rb1, rb) or haveRelationBetween(cfs[i].rb2, rb) then
    begin
			tmpP:= rb.getLocalCoordinatesPoint(cfs[i].cp);
			if tmpP.x < 0 then heelForce:= true;
			if tmpP.x > 0 then toeForce:= true;
		end;
	end;
end;

{ TBehaviourController }

procedure TBehaviourController.setUpperBodyPose(leanSagittal: Double);
var
  curState: TSimBiConState;
  tmpTraj: TTrajectory;
begin
	curState:= lowLCon.states[lowLCon.getFSMState()];
	tmpTraj:= curState.getTrajectory('root');
	if Assigned(tmpTraj) then
		tmpTraj.components[0].offset:= leanSagittal;
	tmpTraj:= curState.getTrajectory('pelvis_lowerback');
	if Assigned(tmpTraj) then
		tmpTraj.components[0].offset:= leanSagittal * 1.5;
	tmpTraj:= curState.getTrajectory('lowerback_torso');
	if Assigned(tmpTraj) then
		tmpTraj.components[0].offset:= leanSagittal * 2.5;
	tmpTraj:= curState.getTrajectory('torso_head');
	if Assigned(tmpTraj) then
		tmpTraj.components[0].offset:= leanSagittal * 3.0;
end;

procedure TBehaviourController.setKneeBend(v: Double; swingAlso: Boolean);
var
  curState: TSimBiConState;
  tmpTraj: TTrajectory;
begin
	curState:= lowLCon.states[lowLCon.FSMStateIndex];

	tmpTraj:= curState.getTrajectory('STANCE_Knee');
	tmpTraj.components[0].offset:= v;

	if swingAlso then
  begin
		tmpTraj:= curState.getTrajectory('SWING_Knee');
		tmpTraj.components[0].offset:= v;
	end;
end;

procedure TBehaviourController.setVelocities(velDS: Double);
begin
  lowLCon.velDSagittal:= velDS;
end;

constructor TBehaviourController.Create(b: TCharacter; llc: TIKVMCController; w: TWorldOracle);
begin
	Self.bip:= b;
	Self.lowLCon:= llc;
	Self.wo:= w;

	//we should estimate these from the character info...
	legLength:= 1;
	ankleBaseHeight:= 0.04;

	stepTime:= 0.6;
end;

destructor TBehaviourController.Destroy;
begin

  inherited;
end;

procedure TBehaviourController.adjustStepHeight;
var
  panicIntensity, foot_sole_y: Double;
begin
	lowLCon.unplannedForHeight:= 0;
	if Assigned(wo) then
  begin
		//the trajectory of the foot was generated without taking the environment into account, so check to see if there are any un-planned bumps (at somepoint in the near future)
		lowLCon.unplannedForHeight:= wo.getWorldHeightAt(bip, lowLCon.swingFoot.GetPosition + lowLCon.swingFoot.GetVelocity * 0.1) * 1.0;
    foot_sole_y:= lowLCon.stanceFoot.getWorldCoordinatesPoint(TVector2.From(0, -lowLCon.stanceFoot.Height / 2)).y;
    lowLCon.unplannedForHeight:= lowLCon.unplannedForHeight + (lowLCon.unplannedForHeight - foot_sole_y) * 0.5;
    TWorldOracle.DebugPoint.y:= lowLCon.unplannedForHeight;
  end;

	//if the foot is high enough, we shouldn't do much about it... also, if we're close to the start or end of the
	//walk cycle, we don't need to do anything... the thing below is a quadratic that is 1 at 0.5, 0 at 0 and 1...
	panicIntensity:= -4 * lowLCon.phi * lowLCon.phi + 4 * lowLCon.phi;
	panicIntensity:= panicIntensity * getPanicLevel();
	lowLCon.panicHeight:= panicIntensity * 0.05;
end;

procedure TBehaviourController.setElbowAngles(leftElbowAngle, rightElbowAngle: Double);
var
  stanceElbowAngle, swingElbowAngle: Double;
  curState: TSimBiConState;
  tmpTraj: TTrajectory;
begin
  stanceElbowAngle:= IfThen(lowLCon.stance = LEFT_STANCE, leftElbowAngle, rightElbowAngle);
	swingElbowAngle:= IfThen(lowLCon.stance = LEFT_STANCE, rightElbowAngle, leftElbowAngle);

	curState:= lowLCon.states[lowLCon.FSMStateIndex];
	tmpTraj:= curState.getTrajectory('STANCE_Elbow');
	if Assigned(tmpTraj) then
		tmpTraj.components[0].offset:= stanceElbowAngle;
	tmpTraj:= curState.getTrajectory('SWING_Elbow');
	if Assigned(tmpTraj) then
		tmpTraj.components[0].offset:= swingElbowAngle * -1;
end;

procedure TBehaviourController.setShoulderAngles(leftSwing, rightSwing: Double);
var
  stanceSwing, swingSwing: Double;
  curState: TSimBiConState;
  tmpTraj: TTrajectory;
begin
	stanceSwing:= IfThen(lowLCon.stance = LEFT_STANCE, leftSwing, rightSwing);
	swingSwing:= IfThen(lowLCon.stance = RIGHT_STANCE, leftSwing, rightSwing);

	curState:= lowLCon.states[lowLCon.FSMStateIndex];
	tmpTraj:= curState.getTrajectory('STANCE_Shoulder');
	if Assigned(tmpTraj) then
		tmpTraj.components[0].offset:= stanceSwing;

	tmpTraj:= curState.getTrajectory('SWING_Shoulder');
	if Assigned(tmpTraj) then
		tmpTraj.components[0].offset:= swingSwing;
end;

procedure TBehaviourController.requestStepTime(stepTime: Double);
begin
  Self.stepTime:= stepTime;
end;

procedure TBehaviourController.requestStepHeight(stepHeight: Double);
begin
  Self.stepHeight:= stepHeight;
end;

procedure TBehaviourController.requestVelocities(velDS: Double);
begin
	velDSagittal:= velDS;
end;

procedure TBehaviourController.requestUpperBodyPose(leanS: Double);
begin
  Self.ubSagittalLean:= leanS;
end;

procedure TBehaviourController.requestKneeBend(kb: Double);
begin
  Self.kneeBend:= kb;
end;

procedure TBehaviourController.setDesiredSwingFootLocation;
const
	dt = 0.001;
var
  step: TVector2;
begin
	step:= computeSwingFootLocationEstimate(lowLCon.comPosition, lowLCon.phi);
	lowLCon.swingFootTrajectorySagittal.setKnotValue(0, step.x);

	step:= computeSwingFootLocationEstimate(lowLCon.comPosition + lowLCon.comVelocity * dt, lowLCon.phi+dt);
	lowLCon.swingFootTrajectorySagittal.setKnotValue(1, step.x);
	//to give some gradient information, here's what the position will be a short time later...

	lowLCon.swingFootTrajectorySagittal.setKnotPosition(0, lowLCon.phi);
	lowLCon.swingFootTrajectorySagittal.setKnotPosition(1, lowLCon.phi+dt);
end;

function TBehaviourController.computeSwingFootLocationEstimate(const comPos: TVector2; phase: Double): TVector2;
var
  step, initialStep: TVector2;
  t: Double;
begin
	step:= lowLCon.computeIPStepLocation();

	step.x:= step.x - lowLCon.velDSagittal / 20;

	boundToRange(step.x, -0.4 * legLength, 0.4 * legLength);

	initialStep:= VectorDelta(comPos, swingFootStartPos);
	//when phi is small, we want to be much closer to where the initial step is - so compute this quantity in character-relative coordinates
	//now interpolate between this position and initial foot position - but provide two estimates in order to provide some gradient information
	t:= (1-phase);
	t:= t * t;
	boundToRange(t, 0, 1);

  result:= TVector2.From(0, 0);
  result:= result + step * (1-t);
  result:= result + initialStep * t;

	result.y:= 0;
end;

procedure TBehaviourController.initializeDefaultParameters;
begin
	lowLCon.updateDAndV();
end;

procedure TBehaviourController.simStepPlan(dt: Double);
begin
  lowLCon.updateSwingAndStanceReferences();
	if lowLCon.phi <= 0.01 then
		swingFootStartPos:= lowLCon.swingFoot.getWorldCoordinatesPoint(bip.joints[lowLCon.swingAnkleIndex].cJPos);

	//compute desired swing foot location...
	setDesiredSwingFootLocation();

	//set some of these settings
	setUpperBodyPose(ubSagittalLean);
	setKneeBend(kneeBend);
	setVelocities(velDSagittal);

	//adjust for panic mode or unplanned terrain...
	adjustStepHeight();
end;

procedure TBehaviourController.conTransitionPlan;
begin
	lowLCon.updateSwingAndStanceReferences();
	lowLCon.updateDAndV();
	lowLCon.states[0].stateTime:= stepTime;

	lowLCon.updateSwingAndStanceReferences();
	swingFootStartPos:= lowLCon.swingFoot.CMPosition;

	//now prepare the step information for the following step:
	lowLCon.swingFootHeightTrajectory.clear();
	lowLCon.swingFootTrajectorySagittal.clear();

	lowLCon.swingFootHeightTrajectory.addKnot(0, ankleBaseHeight);
	lowLCon.swingFootHeightTrajectory.addKnot(0.5, ankleBaseHeight + 0.01 + 0.1 + 0 + stepHeight);
	lowLCon.swingFootHeightTrajectory.addKnot(1, ankleBaseHeight + 0.01);

	lowLCon.swingFootTrajectorySagittal.addKnot(0,0);
	lowLCon.swingFootTrajectorySagittal.addKnot(1,0);
end;

function getValueInFuzzyRange(val, minB, minG, maxG, maxB: Double) : Double; inline;
begin
	if (val <= minB) or (val >= maxB) then
		Exit(1);
	if (val >= minG) and (val <= maxG) then
		Exit(0);
	if (val > minB) and (val < minG) then
		Exit((minG - val) / (minG - minB));
	if (val > maxG) and (val < maxB) then
		Exit((val - maxG) / (maxB - maxG));
	//the input was probably wrong, so return panic...
	Exit(1);
end;

function TBehaviourController.getPanicLevel: Double;
var
  panicEstimate: Double;
begin
	panicEstimate:= getValueInFuzzyRange(lowLCon.v.x, lowLCon.velDSagittal-0.4, lowLCon.velDSagittal-0.1, lowLCon.velDSagittal+0.1, lowLCon.velDSagittal+0.4);
	Result:= panicEstimate/1; // может быть не делить на два т.к. я убрал coronal?
end;

{ TLinearBalanceFeedback }

constructor TLinearBalanceFeedback.Create;
begin
  inherited;
  cd:= 0;
  cv:= 0;
  vMin:= -1000;
  dMin:= -1000;
  vMax:=  1000;
  dMax:=  1000;
end;

function TLinearBalanceFeedback.getFeedbackContribution(con: TSimBiController; j: TJoint; phi: Double; const d, v: TVector2): Double;
const
  feedbackProjectionAxis: TVector2 = (X: 1.0; Y: 0.0);
var
  dToUse, vToUse: Double;
begin
  dToUse:= b2Dot(d, feedbackProjectionAxis);
  vToUse:= b2Dot(v, feedbackProjectionAxis);

  dToUse:= clamp(dToUse, dMin, dMax);
  vToUse:= clamp(vToUse, vMin, vMax);

  Result:= dToUse * cd + vToUse * cv;
end;

end.

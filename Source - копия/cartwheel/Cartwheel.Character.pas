unit Cartwheel.Character;

{$SCOPEDENUMS ON}

interface

uses
  System.SysUtils, System.Math, System.Generics.Collections,
  Box2D.Physics2D, Box2D.Physics2DTypes;

type
  TArticulatedFigure = class;
  TArticulatedRigidBody = class;

  TRigidBody = class
  public
    Name: String;
    Body: Tb2Body;
    constructor Create(const Name: String; Body: Tb2Body);

    function GetMass : Double;

    function GetOrientation : Double;
    procedure SetOrientation(Orientation: Double);

    function GetPosition : TVector2;
    procedure SetPosition(const Position: TVector2);

    function GetVelocity : TVector2;
    function GetAngularVelocity : Double;

    function getWorldCoordinatesVector(const localVector: TVector2) : TVector2;
    function getWorldCoordinatesPoint(const localPoint: TVector2) : TVector2;

    function getLocalCoordinatesVector(const globalVector: TVector2) : TVector2;
    function getLocalCoordinatesPoint(const globalPoint: TVector2) : TVector2;

    function getAbsoluteVelocityForLocalPoint(const localPoint: TVector2) : TVector2;

    property CMPosition : TVector2 read GetPosition write SetPosition;
    property CMVelocity : TVector2 read GetVelocity;
  end;

  TJoint = class
  private
    function GetChild: TArticulatedRigidBody;
    function GetChildJPos: TVector2;
    function GetParent: TArticulatedRigidBody;
    function GetParentJPos: TVector2;
    function GetUseJointLimits: Boolean;
    procedure SetUseJointLimits(Value: Boolean);
  public
    Joint: Tb2RevoluteJoint;
    torque: Double;
    Name: String;
    Id: Integer;
    constructor Create(const Name: String; Joint: Tb2RevoluteJoint);
    destructor Destroy; override;

    property parent: TArticulatedRigidBody read GetParent;
    property pJPos : TVector2 read GetParentJPos;
    property child : TArticulatedRigidBody read GetChild;
    property cJPos : TVector2 read GetChildJPos;
    property useJointLimits : Boolean read GetUseJointLimits write SetUseJointLimits;

    procedure computeRelativeOrientation(var aRel: Double);
    procedure fixJointConstraints;

    procedure setTorque(t: Double);

    property ParentJointPosition : TVector2 read GetParentJPos;
  end;

  TBodyPart =
  (
    TorsoAndHead,
    Arms,
    Legs
  );

  TArticulatedRigidBody = class(TRigidBody)
  public
    parentJoint: TJoint;
    childJoints: TList<TJoint>;
    AFParent: TArticulatedFigure;
    BodyPart: TBodyPart;
    Width, Height: Double;
    constructor Create(const Name: String; BodyPart: TBodyPart; Body: Tb2Body; Width, Height: Double);
    destructor Destroy; override;
  end;

  TContactPoint = record
    cp: TVector2; // точка в пространстве
    // два тела
    rb1, rb2: TArticulatedRigidBody;
    // сила примененная к первому телу (ко второму тела применена обратная сила)
    f: TVector2;
  end;

  TArticulatedFigure = class
  strict private
    FRoot: TArticulatedRigidBody;
    procedure SetRoot(Root: TArticulatedRigidBody);
    procedure computeMass;
    procedure setJointsIndexes;
    function GetJointCount: Integer;
  public
    Name: String;
    Mass: Double;

    Joints: TList<TJoint>;
    ARBs: TList<TArticulatedRigidBody>;

    Contacts: TList<TContactPoint>;

    constructor Create;
    destructor Destroy; override;

    property Root : TArticulatedRigidBody read FRoot write SetRoot;
    procedure addArticulatedRigidBody(arb: TArticulatedRigidBody);
    procedure addJoint(joint_disown: TJoint);
    procedure addContact(const Contact: TContactPoint);
    procedure clearContacts;

    procedure addJointsToList(otherJoints: TList<TJoint>);

    procedure fixJointConstraints;
    procedure completeFigure;

    property JointCount : Integer read GetJointCount;
    function getJoint(i: Integer) : TJoint;
    function getJointIndex(joint: TJoint) : Integer; overload;
    function getJointIndex(const jName: String) : Integer; overload;
    function getJointByName(const jName: String) : TJoint;

    function getARBByName(const Name: String) : TArticulatedRigidBody;
  end;

  TCharacter = class(TArticulatedFigure)
  public
    function getCOM : TVector2;
    function getCOMVelocity : TVector2;
  end;

function VectorDelta(const V1, V2: TVector2) : TVector2;

implementation

function VectorDelta(const V1, V2: TVector2) : TVector2;
begin
  Result:= V2 - V1;
end;

{ TArticulatedFigure }

constructor TArticulatedFigure.Create;
begin
  Joints:= TList<TJoint>.Create;
  ARBs:= TList<TArticulatedRigidBody>.Create;
  Contacts:= TList<TContactPoint>.Create;
end;

destructor TArticulatedFigure.Destroy;
var
  Joint: TJoint;
  Body: TArticulatedRigidBody;
begin
  FreeAndNil(Contacts);
  for Joint in Joints do
    Joint.Free;
  FreeAndNil(Joints);
  for Body in ARBs do
    Body.Free;
  FreeAndNil(ARBs);
  inherited;
end;

procedure TArticulatedFigure.SetRoot(Root: TArticulatedRigidBody);
begin
	if Self.FRoot <> nil then
		raise Exception.Create('This articulated figure already has a root');
  Root.AFParent:= Self;
	Self.FRoot:= Root;
end;

procedure TArticulatedFigure.addArticulatedRigidBody(arb: TArticulatedRigidBody);
begin
  arb.AFParent:= Self;
	arbs.Add(arb);
end;

procedure TArticulatedFigure.addJoint(joint_disown: TJoint);
begin
  joints.Add(joint_disown);
end;

procedure TArticulatedFigure.addContact(const Contact: TContactPoint);
begin
  Contacts.Add(Contact);
end;

procedure TArticulatedFigure.clearContacts;
begin
  Contacts.Clear;
end;

procedure TArticulatedFigure.computeMass;
var
  curMass, totalMass: Double;
  i: Integer;
begin
	curMass:= root.getMass();
	totalMass:= curMass;

	for i:=0 to joints.Count - 1 do
  begin
		curMass:= joints[i].child.getMass();
		totalMass:= totalMass + curMass;
	end;

	mass:= totalMass;
end;

procedure TArticulatedFigure.addJointsToList(otherJoints: TList<TJoint>);
var
  i: Integer;
begin
	for i:=0 to joints.Count - 1 do
		otherJoints.Add(joints[i]);
end;

function TArticulatedFigure.GetJointCount: Integer;
begin
  Result:= Joints.Count;
end;

function TArticulatedFigure.getJoint(i: Integer): TJoint;
begin
  if (i < 0) or (i > joints.Count - 1) then
    Exit(nil);
  Result:= Joints[i];
end;

function TArticulatedFigure.getJointIndex(joint: TJoint) : Integer;
begin
  if joint = nil then Result:= -1
  else
  if getJoint(joint.Id) = joint then Result:= joint.Id
  else Result:= -1;
end;

function TArticulatedFigure.getJointIndex(const jName: String): Integer;
var
  i: Integer;
begin
  for i:=0 to joints.Count - 1 do
    if joints[i].name = jName then
      Exit(i);
  Result:= -1;
end;

function TArticulatedFigure.getJointByName(const jName: String): TJoint;
var
  i: Integer;
begin
  for i:=0 to joints.Count - 1 do
    if joints[i].name = jName then
      Exit(joints[i]);
  Result:= nil;
end;

procedure TArticulatedFigure.fixJointConstraints;
var
  i: Integer;
begin
	if not Assigned(root) then Exit;

	for i:=0 to root.childJoints.Count - 1 do
		root.childJoints[i].fixJointConstraints;
end;

procedure TArticulatedFigure.completeFigure;
begin
  setJointsIndexes;
  computeMass;
  fixJointConstraints;
end;

procedure TArticulatedFigure.setJointsIndexes;
var
  i: Integer;
begin
  for i := 0 to Joints.Count - 1 do
    Joints[i].Id:= i;
end;

function TArticulatedFigure.getARBByName(const Name: String): TArticulatedRigidBody;
var
  i: Integer;
begin
  if Assigned(root) and (root.name = name) then
    Exit(root);

  for i:=0 to arbs.Count - 1 do
    if arbs[i].name = name then
      Exit(arbs[i]);

  Exit(nil);
end;

{ TCharacter }

function TCharacter.getCOM: TVector2;
var
  curMass, totalMass: Double;
  COM: TVector2;
  i: Integer;
begin
	curMass:= root.getMass();
	totalMass:= curMass;
	COM:= root.CMPosition * curMass;
	for i:=0 to joints.Count - 1 do
  begin
		curMass:= joints[i].child.getMass();
		totalMass:= totalMass + curMass;
		COM:= COM + joints[i].child.CMPosition * curMass;
	end;

	COM:= COM / totalMass;

	Result:= COM;
end;

function TCharacter.getCOMVelocity: TVector2;
var
  curMass, totalMass: Double;
  COMVel: TVector2;
  i: Integer;
begin
	curMass:= root.getMass();
	totalMass:= curMass;
	COMVel:= root.CMVelocity * curMass;
	for i:=0 to joints.Count - 1 do
  begin
		curMass:= joints[i].child.getMass();
		totalMass:= totalMass + curMass;
		COMVel:= COMVel + joints[i].child.CMVelocity * curMass;
	end;

	COMVel:= COMVel / totalMass;

	Result:= COMVel;
end;

{ TRigidBody }

constructor TRigidBody.Create(const Name: String; Body: Tb2Body);
begin
  Self.Name:= Name;
  Self.Body:= Body;
  Body.userData:= Self;
end;

function TRigidBody.GetMass: Double;
begin
  Result:= Body.GetMass;
end;

function TRigidBody.GetOrientation: Double;
begin
  Result:= Body.GetAngle;
end;

procedure TRigidBody.SetOrientation(Orientation: Double);
begin
  Body.SetTransform(Body.GetPosition, Orientation);
end;

function TRigidBody.GetPosition: TVector2;
begin
  Result:= Body.GetPosition;
end;

procedure TRigidBody.SetPosition(const Position: TVector2);
begin
  Body.SetTransform(Position, Body.GetAngle);
end;

function TRigidBody.getWorldCoordinatesVector(const localVector: TVector2): TVector2;
var
  Rot: Tb2Rot;
begin
  Rot.SetAngle(Self.GetOrientation);
  Result:= b2Mul(Rot, localVector);
end;

function TRigidBody.getWorldCoordinatesPoint(const localPoint: TVector2): TVector2;
begin
  Result:= Self.Body.GetPosition + getWorldCoordinatesVector(localPoint);
end;

function TRigidBody.getLocalCoordinatesVector(const globalVector: TVector2): TVector2;
var
  Rot: Tb2Rot;
begin
  Rot.SetAngle(-Self.GetOrientation);
  Result:= b2Mul(Rot, globalVector);
end;

function TRigidBody.getLocalCoordinatesPoint(const globalPoint: TVector2): TVector2;
begin
	Result:= getLocalCoordinatesVector(globalPoint) - getLocalCoordinatesVector(Self.GetPosition);
end;

function TRigidBody.getAbsoluteVelocityForLocalPoint(const localPoint: TVector2): TVector2;
begin
	Result:= b2Cross(GetAngularVelocity, getWorldCoordinatesVector(localPoint)) + GetVelocity;
end;

function TRigidBody.GetVelocity: TVector2;
begin
  Result:= Body.GetLinearVelocity;
end;

function TRigidBody.GetAngularVelocity: Double;
begin
  Result:= Body.GetAngularVelocity;
end;

{ TArticulatedRigidBody }

constructor TArticulatedRigidBody.Create(const Name: String; BodyPart: TBodyPart; Body: Tb2Body; Width, Height: Double);
begin
  inherited Create(Name, Body);
  Self.BodyPart:= BodyPart;
  childJoints:= TList<TJoint>.Create;
  Self.Width:= Width;
  Self.Height:= Height;
end;

destructor TArticulatedRigidBody.Destroy;
begin
  // todo remove Box2D body from world
  FreeAndNil(childJoints);
  inherited;
end;

{ TJoint }

constructor TJoint.Create(const Name: String; Joint: Tb2RevoluteJoint);
begin
  Self.Name:= Name;
  Self.Joint:= Joint;
  Parent.childJoints.Add(Self);
  Child.parentJoint:= Self;
end;

destructor TJoint.Destroy;
begin
  // todo remove Box2D joint from world
  inherited;
end;

function TJoint.GetParent: TArticulatedRigidBody;
begin
  Result:= TObject(Joint.GetBodyA.UserData) as TArticulatedRigidBody;
end;

function TJoint.GetParentJPos: TVector2;
begin
  Result:= Joint.GetLocalAnchorA;
end;

function TJoint.GetChild: TArticulatedRigidBody;
begin
  Result:= TObject(Joint.GetBodyB.UserData) as TArticulatedRigidBody;
end;

function TJoint.GetChildJPos: TVector2;
begin
  Result:= Joint.GetLocalAnchorB;
end;

function TJoint.GetUseJointLimits: Boolean;
begin
  Result:= Joint.IsLimitEnabled;
end;

procedure TJoint.SetUseJointLimits(Value: Boolean);
begin
  Joint.EnableLimit(Value);
end;

procedure TJoint.computeRelativeOrientation(var aRel: Double);
begin
  aRel:= -parent.GetOrientation + child.GetOrientation;
end;

procedure TJoint.fixJointConstraints;
var
  rc, rp: TVector2;
  i: Integer;
begin
	if not Assigned(child) then Exit;

	//if it has a parent, we will assume that the parent's position is correct, and move the children to satisfy the joint constraint
	if Assigned(parent) then
  begin
		//now worry about the joint positions

		//compute the vector rc from the child's joint position to the child's center of mass (in world coordinates)
		rc:= child.getWorldCoordinatesVector(VectorDelta(cJPos, TVector2.From(0, 0)));
		//and the vector rp that represents the same quanity but for the parent
		rp:= parent.getWorldCoordinatesVector(VectorDelta(pJPos, TVector2.From(0, 0)));

		//the location of the child's CM is now: pCM - rp + rc
		child.SetPosition(parent.GetPosition + (rc - rp));
	end;

	//make sure that we recursivley fix all the other joint constraints in the articulated figure
  for i:=0 to child.childJoints.Count - 1 do
    child.childJoints[i].fixJointConstraints;
end;

procedure TJoint.setTorque(t: Double);
begin
  torque:= t;
end;

end.

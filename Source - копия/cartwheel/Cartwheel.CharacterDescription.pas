unit Cartwheel.CharacterDescription;

interface

uses
  System.Math,
  Box2D.Physics2D, Box2D.Physics2DTypes,
  Cartwheel.Character;

type
  TPoint3D = record
    X, Y, Z: Double;
    constructor Create(X, Y, Z: Double);
  end;

  TMinMax = record
  strict private
    _minValue, _maxValue: Double;
  private
    function clamp(value: Double) : Double;
    constructor Create(minValue: Double; maxValue: Double);
  end;

  TValue = record
  strict private
    MinMax: TMinMax;
    _value: Double;
  private
    procedure SetValue(Value: Double);
    function get : Double;
  public
    constructor Create(value: Double; minValue: Double = NAN; maxValue: Double = NAN);
    property value : Double read _value write SetValue;
    class operator Implicit(const Value: TValue) : Double; static;
  end;

  TSymmetric = record
  strict private
    MinMax: TMinMax;
    _side: array [0..2 - 1] of Double;
  private
    function GetSide(side: Integer): Double;
    procedure SetSide(side: Integer; value: Double);
    procedure SetLeft(value: Double);
    procedure SetRight(value: Double);
  public
    constructor Create(value: Double; minValue: Double = NAN; maxValue: Double = NAN);
    property Side[side: Integer] : Double read GetSide write SetSide; default;
    property Right : Double read _side[0] write SetRight;
    property Left : Double read _side[1] write SetLeft;
    procedure forceSymmetric;
  end;

  TCharacterDescription = class
  strict private
    const
      hu = 0.2286;
    var
      _isSymmetric, _indirectVarsAreValid: Boolean;

      // native variables
      _footSizeX: TSymmetric;
      _footSizeZ: TSymmetric;
      _ankleRelativePosY: TSymmetric;
      _lowerLegDiameter: TSymmetric;
      _upperLegDiameter: TSymmetric;
      _legSizeY: TValue;
      _kneeRelativePosY: TSymmetric;
      _legRelativeAnchorX: TSymmetric;
      _pelvisDiameter: TValue;
      _torsoDiameter: TValue;
      _trunkSizeY: TValue;
      _waistRelativePosY: TValue;
      _chestRelativePosY: TValue;
      _neckSizeY: TValue;
      _headSizeX: TValue;
      _headSizeY: TValue;
      _headSizeZ: TValue;

      _upperArmDiameter: TSymmetric;
      _lowerArmDiameter: TSymmetric;
      _armSizeY: TSymmetric;
      _elbowRelativePosY: TSymmetric;

      _groundPosY: TValue;

      // indirect variables
      _legPosX: TSymmetric;
      _anklePosY: TSymmetric;
      _kneePosY: TSymmetric;
      _hipPosY: TValue;
      _waistPosY: TValue;
      _chestPosY: TValue;
      _shoulderPosY: TValue;
      _neckPosY: TValue;
      _armPosX: TSymmetric;
      _elbowPosY: TSymmetric;
      _wristPosY: TSymmetric;

    function GetFootSizeX(side: Integer) : Double;
    procedure SetFootSizeX(side: Integer; value: Double);
    function GetFootSizeZ(side: Integer) : Double;
    procedure SetFootSizeZ(side: Integer; value: Double);
    function GetAnkleRelativePosY(side: Integer) : Double;
    procedure SetAnkleRelativePosY(side: Integer; value: Double);
    function GetLowerLegDiameter(side: Integer) : Double;
    procedure SetLowerLegDiameter(side: Integer; value: Double);
    function GetUpperLegDiameter(side: Integer) : Double;
    procedure SetUpperLegDiameter(side: Integer; value: Double);
    function GetLegSizeY : Double;
    procedure SetLegSizeY(value: Double);
    function GetKneeRelativePosY(side: Integer) : Double;
    procedure SetKneeRelativePosY(side: Integer; value: Double);
    function GetLegRelativeAnchorX(side: Integer) : Double;
    procedure SetLegRelativeAnchorX(side: Integer; value: Double);
    function GetPelvisDiameter : Double;
    procedure SetPelvisDiameter(value: Double);
    function GetTorsoDiameter : Double;
    procedure SetTorsoDiameter(value: Double);
    function GetTrunkSizeY : Double;
    procedure SetTrunkSizeY(value: Double);
    function GetWaistRelativePosY : Double;
    procedure SetWaistRelativePosY(value: Double);
    function GetChestRelativePosY : Double;
    procedure SetChestRelativePosY(value: Double);
    function GetNeckSizeY : Double;
    procedure SetNeckSizeY(value: Double);
    function GetHeadSizeX : Double;
    procedure SetHeadSizeX(value: Double);
    function GetHeadSizeY : Double;
    procedure SetHeadSizeY(value: Double);
    function GetHeadSizeZ : Double;
    procedure SetHeadSizeZ(value: Double);
    function GetUpperArmDiameter(side: Integer) : Double;
    procedure SetUpperArmDiameter(side: Integer; value: Double);
    function GetLowerArmDiameter(side: Integer) : Double;
    procedure SetLowerArmDiameter(side: Integer; value: Double);
    function GetArmSizeY(side: Integer) : Double;
    procedure SetArmSizeY(side: Integer; value: Double);
    function GetElbowRelativePosY(side: Integer) : Double;
    procedure SetElbowRelativePosY(side: Integer; value: Double);
    function GetGroundPosY : Double;
    procedure SetGroundPosY(value: Double);

    function GetLegPosX(side: Integer) : Double;
    procedure ComputeLegPosX(side: Integer);
    procedure SetLegPosX(side: Integer; value: Double);
    function GetAnklePosY(side: Integer) : Double;
    procedure ComputeAnklePosY(side: Integer);
    procedure SetAnklePosY(side: Integer; value: Double);
    function GetKneePosY(side: Integer) : Double;
    procedure ComputeKneePosY(side: Integer);
    procedure SetKneePosY(side: Integer; value: Double);
    function GetHipPosY : Double;
    procedure ComputeHipPosY;
    procedure SetHipPosY(value: Double);
    function GetWaistPosY : Double;
    procedure ComputeWaistPosY;
    procedure SetWaistPosY(value: Double);
    function GetChestPosY : Double;
    procedure ComputeChestPosY;
    procedure SetChestPosY(value: Double);
    function GetShoulderPosY : Double;
    procedure ComputeShoulderPosY;
    procedure SetShoulderPosY(value: Double);
    function GetNeckPosY : Double;
    procedure ComputeNeckPosY;
    procedure SetNeckPosY(value: Double);
    function GetArmPosX(side: Integer) : Double;
    procedure ComputeArmPosX(side: Integer);
    function GetElbowPosY(side: Integer) : Double;
    procedure ComputeElbowPosY(side: Integer);
    procedure SetElbowPosY(side: Integer; value: Double);
    function GetWristPosY(side: Integer) : Double;
    procedure ComputeWristPosY(side: Integer);
    procedure SetWristPosY(side: Integer; value: Double);

    procedure computeIndirectVars;

    procedure setSymmetric(value: Boolean);

    function createArticulatedBox(world: Tb2World; const Name: String; BodyPart: TBodyPart; SizeX, SizeY, SizeZ, mass: Double;
      moiScale: Double = 1; PosX: Double = 0; PosY: Double = 0) : TArticulatedRigidBody;
    function createArticulatedEllipsoid(world: Tb2World; const Name: String; BodyPart: TBodyPart; RadiusX, RadiusY, RadiusZ, mass: Double;
      moiScale: Double = 1; PosX: Double = 0; PosY: Double = 0) : TArticulatedRigidBody;
    function createArticulatedCylinder(world: Tb2World; const Name: String; BodyPart: TBodyPart; Height, Radius, mass: Double;
      moiScale: Double = 1; PosX: Double = 0; PosY: Double = 0) : TArticulatedRigidBody;

    function createJoint(world: Tb2World; const Name: String; const posInParent, posInChild: TPoint3D; minAngle, maxAngle: Double;
      Parent, Child: TArticulatedRigidBody) : TJoint;
  public
    constructor Create;

    // native variables
    property FootSizeX[side: Integer]: Double read GetFootSizeX write SetFootSizeX;
    property FootSizeZ[side: Integer]: Double read GetFootSizeZ write SetFootSizeZ;
    property AnkleRelativePosY[side: Integer]: Double read GetAnkleRelativePosY write SetAnkleRelativePosY;
    property LowerLegDiameter[side: Integer]: Double read GetLowerLegDiameter write SetLowerLegDiameter;
    property UpperLegDiameter[side: Integer]: Double read GetUpperLegDiameter write SetUpperLegDiameter;
    property LegSizeY: Double read GetLegSizeY write SetLegSizeY;
    property KneeRelativePosY[side: Integer]: Double read GetKneeRelativePosY write SetKneeRelativePosY;
    property LegRelativeAnchorX[side: Integer]: Double read GetLegRelativeAnchorX write SetLegRelativeAnchorX;
    property PelvisDiameter: Double read GetPelvisDiameter write SetPelvisDiameter;
    property TorsoDiameter: Double read GetTorsoDiameter write SetTorsoDiameter;
    property TrunkSizeY: Double read GetTrunkSizeY write SetTrunkSizeY;
    property WaistRelativePosY: Double read GetWaistRelativePosY write SetWaistRelativePosY;
    property ChestRelativePosY: Double read GetChestRelativePosY write SetChestRelativePosY;
    property NeckSizeY: Double read GetNeckSizeY write SetNeckSizeY;
    property HeadSizeX: Double read GetHeadSizeX write SetHeadSizeX;
    property HeadSizeY: Double read GetHeadSizeY write SetHeadSizeY;
    property HeadSizeZ: Double read GetHeadSizeZ write SetHeadSizeZ;

    property UpperArmDiameter[side: Integer]: Double read GetUpperArmDiameter write SetUpperArmDiameter;
    property LowerArmDiameter[side: Integer]: Double read GetLowerArmDiameter write SetLowerArmDiameter;
    property ArmSizeY[side: Integer]: Double read GetArmSizeY write SetArmSizeY;
    property ElbowRelativePosY[side: Integer]: Double read GetElbowRelativePosY write SetElbowRelativePosY;

    property GroundPosY: Double read GetGroundPosY write SetGroundPosY;

    // indirect variables
    property LegPosX[side: Integer]: Double read GetLegPosX write SetLegPosX;
    property AnklePosY[side: Integer]: Double read GetAnklePosY write SetAnklePosY;
    property KneePosY[side: Integer]: Double read GetKneePosY write SetKneePosY;
    property HipPosY: Double read GetHipPosY write SetHipPosY;
    property WaistPosY: Double read GetWaistPosY write SetWaistPosY;
    property ChestPosY: Double read GetChestPosY write SetChestPosY;
    property ShoulderPosY: Double read GetShoulderPosY write SetShoulderPosY;
    property NeckPosY: Double read GetNeckPosY write SetNeckPosY;
    property ArmPosX[side: Integer]: Double read GetArmPosX;
    property ElbowPosY[side: Integer]: Double read GetElbowPosY write SetElbowPosY;
    property WristPosY[side: Integer]: Double read GetWristPosY write SetWristPosY;

    property isSymmetric : Boolean read _isSymmetric write setSymmetric;

    function createCharacter(world: Tb2World; const Pos: TVector2) : TCharacter;
  end;

implementation

{ TPoint3D }

constructor TPoint3D.Create(X, Y, Z: Double);
begin
  Self.X:= X;
  Self.Y:= Y;
  Self.Z:= Z;
end;

{ TCharacterDescription }

constructor TCharacterDescription.Create;
begin
  self._isSymmetric:= True;
  self._indirectVarsAreValid:= False;

  // native
  _footSizeX:= TSymmetric.Create(0.45 * hu, 0.6*hu, hu);
  _footSizeZ:= TSymmetric.Create(1 * hu, 0.1*hu);
  _ankleRelativePosY:= TSymmetric.Create(0.05, 0.05, 0.3);
  _lowerLegDiameter:= TSymmetric.Create(0.33 * hu, 0.2*hu, hu);
  _upperLegDiameter:= TSymmetric.Create(0.4 * hu, 0.2*hu, hu);
  _legSizeY:= TValue.Create(4*hu, 2*hu, 6*hu);
  _kneeRelativePosY:= TSymmetric.Create(0.52, 0.2, 0.8);
  _legRelativeAnchorX:= TSymmetric.Create(0.6, 0.2, 0.8);
  _pelvisDiameter:= TValue.Create(1.1 * hu, 0.1*hu, 2.5*hu);
  _torsoDiameter:= TValue.Create(1.4 * hu, hu, 2.6*hu);
  _trunkSizeY:= TValue.Create(2.66 * hu, 1.8*hu, 3.5*hu);
  _waistRelativePosY:= TValue.Create(0.17, 0.1, 0.4);
  _chestRelativePosY:= TValue.Create(0.5, 0.2, 0.8);
  _neckSizeY:= TValue.Create(0.05 * hu, 0.01*hu, 2*hu);
  _headSizeX:= TValue.Create(0.9 * hu, 0.1*hu, 2*hu);
  _headSizeY:= TValue.Create(1.1 * hu, 0.1*hu, 2*hu);
  _headSizeZ:= TValue.Create(1.0 * hu, 0.1*hu, 2*hu);

  _upperArmDiameter:= TSymmetric.Create(0.35 * hu, 0.2*hu, hu);
  _lowerArmDiameter:= TSymmetric.Create(0.28 * hu, 0.2*hu, hu);
  _armSizeY:= TSymmetric.Create(2.7 * hu, 1*hu, 5*hu);
  _elbowRelativePosY:= TSymmetric.Create(0.44444444, 0.01, 0.99);

  // indirect
  _legPosX:= TSymmetric.Create(0);
  _groundPosY:= TValue.Create(0);
  _anklePosY:= TSymmetric.Create(0);
  _kneePosY:= TSymmetric.Create(0);
  _hipPosY:= TValue.Create(0);
  _waistPosY:= TValue.Create(0);
  _chestPosY:= TValue.Create(0);
  _shoulderPosY:= TValue.Create(0);
  _neckPosY:= TValue.Create(0);
  _armPosX:= TSymmetric.Create(0);
  _elbowPosY:= TSymmetric.Create(0);
  _wristPosY:= TSymmetric.Create(0);
end;

procedure TCharacterDescription.computeIndirectVars;
const
  Sides : array [0..2 - 1] of Integer = (-1, 1);
var
  side: Integer;
begin
  if self._indirectVarsAreValid then Exit;

  self.GroundPosY:= 0;
  self.computeHipPosY;
  self.computeShoulderPosY;
  self.computeWaistPosY;
  self.computeChestPosY;
  self.computeNeckPosY;

  for side in Sides do
  begin
    self.computeLegPosX(side);

    self.computeAnklePosY(side);
    self.computeKneePosY(side);

    self.computeArmPosX(side);
    self.computeWristPosY(side);
    self.computeElbowPosY(side);
  end;
end;

procedure TCharacterDescription.setSymmetric(value: Boolean);

  procedure LoopBlock(var val: TSymmetric);
  begin
    val.forceSymmetric;
    self._indirectVarsAreValid:= False;
  end;

begin
  if value = self._isSymmetric then Exit;
  self._isSymmetric:= value;

  LoopBlock(_footSizeX);
  LoopBlock(_footSizeZ);
  LoopBlock(_ankleRelativePosY);
  LoopBlock(_lowerLegDiameter);
  LoopBlock(_upperLegDiameter);
  LoopBlock(_kneeRelativePosY);
  LoopBlock(_legRelativeAnchorX);
  LoopBlock(_upperArmDiameter);
  LoopBlock(_lowerArmDiameter);
  LoopBlock(_armSizeY);
  LoopBlock(_elbowRelativePosY);
end;

function TCharacterDescription.createCharacter(world: Tb2World; const Pos: TVector2): TCharacter;
const
  massScale = 900;
var
  character: TCharacter;

  joint: TJoint;

  pelvisSizeY, pelvisBottomPos, pelvisTopPos, pelvisRadius, rootPosY: Double;
  pelvis: TArticulatedRigidBody;

  totalLowerBackSizeY, lowerBackOffsetY, lowerBackSizeX, lowerBackSizeY, lowerBackSizeZ: Double;
  lowerback: TArticulatedRigidBody;

  totalTorsoSizeY, torsoOffsetY, torsoSizeX, torsoSizeY, torsoSizeZ: Double;
  torso: TArticulatedRigidBody;

  headOffsetY, headSizeX, headSizeY, headSizeZ: Double;
  head: TArticulatedRigidBody;

  leftUpperArmSizeY, leftUpperArmDiameter: Double;
  lUpperArm: TArticulatedRigidBody;

  rightUpperArmSizeY, rightUpperArmDiameter: Double;
  rUpperArm: TArticulatedRigidBody;

  leftLowerArmSizeY, leftLowerArmDiameter: Double;
  lLowerArm: TArticulatedRigidBody;

  rightLowerArmSizeY, rightLowerArmDiameter: Double;
  rLowerArm: TArticulatedRigidBody;

  leftUpperLegSizeY, leftUpperLegDiameter: Double;
  lUpperLeg: TArticulatedRigidBody;

  rightUpperLegSizeY, rightUpperLegDiameter: Double;
  rUpperLeg: TArticulatedRigidBody;

  leftLowerLegSizeY, leftLowerLegDiameter: Double;
  lLowerLeg: TArticulatedRigidBody;

  rightLowerLegSizeY, rightLowerLegDiameter: Double;
  rLowerLeg: TArticulatedRigidBody;

  leftFootSizeX, leftFootSizeY, leftFootSizeZ: Double;
  lFoot: TArticulatedRigidBody;

  rightFootSizeX, rightFootSizeY, rightFootSizeZ: Double;
  rFoot: TArticulatedRigidBody;

  leftToesSizeX, leftToesSizeY, leftToesSizeZ: Double;
  lToes: TArticulatedRigidBody;

  rightToesSizeX, rightToesSizeY, rightToesSizeZ: Double;
  rToes: TArticulatedRigidBody;
begin
  character:= TCharacter.Create;

  character.Name:= 'Instant Character';

  self.GroundPosY:= Pos.y;

  pelvisSizeY:= self.getWaistPosY() - self.getHipPosY();
  pelvisBottomPos:= -pelvisSizeY/2.0-self.getLegSizeY()*0.1;
  pelvisTopPos:= pelvisSizeY/2.0;
  pelvisRadius:= self.getPelvisDiameter()/2.0;
  rootPosY:= self.getHipPosY() + pelvisSizeY/2.0 + 0.007;
  pelvis:= createArticulatedBox(world, 'pelvis', TBodyPart.TorsoAndHead, pelvisRadius*2.0, pelvisSizeY*1.5, pelvisRadius*1.2, -massScale, 3,
    Pos.x, rootPosY);
  character.Root:= pelvis;

  totalLowerBackSizeY:= self.getChestPosY() - self.getWaistPosY();
  lowerBackOffsetY:= 0;
  lowerBackSizeX:= self.getTorsoDiameter() * 0.7;
  lowerBackSizeY:= totalLowerBackSizeY - lowerBackOffsetY;
  lowerBackSizeZ:= lowerBackSizeX * 0.7;
  lowerback:= createArticulatedBox(world, 'lowerBack', TBodyPart.TorsoAndHead, lowerBackSizeX, lowerBackSizeY, lowerBackSizeZ, -massScale);
  character.addArticulatedRigidBody(lowerback);

  joint:= createJoint(world, 'pelvis_lowerback', TPoint3D.Create(0, pelvisSizeY/2.0, 0),
    TPoint3D.Create(0, -lowerBackSizeY/2.0 -lowerBackOffsetY, 0), -1.6, 1.6, pelvis, lowerback);
  character.addJoint(joint);

  totalTorsoSizeY:= self.getShoulderPosY() - self.getChestPosY();
  torsoOffsetY:= -0.2 * totalTorsoSizeY;
  torsoSizeX:= self.getTorsoDiameter();
  torsoSizeY:= totalTorsoSizeY - torsoOffsetY;
  torsoSizeZ:= torsoSizeX * 0.6;
  torso:= createArticulatedBox(world, 'torso', TBodyPart.TorsoAndHead, torsoSizeX, torsoSizeY, torsoSizeZ, -massScale);
  character.addArticulatedRigidBody(torso);

  joint:= createJoint(world, 'lowerback_torso', TPoint3D.Create(0, lowerBackSizeY/2.0, 0),
    TPoint3D.Create(0, -torsoSizeY/2.0 -torsoOffsetY, 0), -1.6, 1.6, lowerback, torso);
  character.addJoint(joint);

  headOffsetY:= self.getNeckSizeY();
  headSizeX:= self.getHeadSizeX();
  headSizeY:= self.getHeadSizeY();
  headSizeZ:= self.getHeadSizeZ();
  head:= createArticulatedEllipsoid(world, 'head', TBodyPart.TorsoAndHead, headSizeX/2.0, headSizeY/2.0, headSizeZ/2.0, -massScale);
  character.addArticulatedRigidBody(head);

  joint:= createJoint(world, 'torso_head', TPoint3D.Create(0, torsoSizeY/2.0, 0),
    TPoint3D.Create(0, -headSizeY/2.0 - headOffsetY, 0), -1.6, 1.6, torso, head);
  character.addJoint(joint);

  leftUpperArmSizeY:= self.getShoulderPosY() - self.getElbowPosY(1);
  leftUpperArmDiameter:= self.getUpperArmDiameter(1);
  lUpperArm:= createArticulatedCylinder(world, 'lUpperArm', TBodyPart.Arms, leftUpperArmSizeY, leftUpperArmDiameter/2.0, -massScale, 3);
  character.addArticulatedRigidBody(lUpperArm);

  joint:= createJoint(world, 'lShoulder', TPoint3D.Create(torsoSizeX*0.52, torsoSizeY*0.32, 0),
    TPoint3D.Create(0, leftUpperArmSizeY/2.0, 0), -1.5, 3.14, torso, lUpperArm);
  character.addJoint(joint);

  rightUpperArmSizeY:= self.getShoulderPosY() - self.getElbowPosY(-1);
  rightUpperArmDiameter:= self.getUpperArmDiameter(-1);
  rUpperArm:= createArticulatedCylinder(world, 'rUpperArm', TBodyPart.Arms, rightUpperArmSizeY, rightUpperArmDiameter/2.0, -massScale, 3);
  character.addArticulatedRigidBody(rUpperArm);

  joint:= createJoint(world, 'rShoulder', TPoint3D.Create(-torsoSizeX*0.52, torsoSizeY*0.32, 0),
    TPoint3D.Create(0, rightUpperArmSizeY/2.0, 0), -1.5, 3.14, torso, rUpperArm);
  character.addJoint(joint);

  leftLowerArmSizeY:= self.getElbowPosY(1) - self.getWristPosY(1);
  leftLowerArmDiameter:= self.getLowerArmDiameter(1);
  lLowerArm:= createArticulatedCylinder(world, 'lLowerArm', TBodyPart.Arms, leftLowerArmSizeY, leftLowerArmDiameter/2.0, -massScale, 3);
  character.addArticulatedRigidBody(lLowerArm);

  joint:= createJoint(world, 'lElbow', TPoint3D.Create(0, -leftUpperArmSizeY/2.0, 0),
     TPoint3D.Create(0, leftLowerArmSizeY/2.0, 0), 0, 2.7, lUpperArm, lLowerArm);
  character.addJoint(joint);

  rightLowerArmSizeY:= self.getElbowPosY(-1) - self.getWristPosY(-1);
  rightLowerArmDiameter:= self.getLowerArmDiameter(-1);
  rLowerArm:= createArticulatedCylinder(world, 'rLowerArm', TBodyPart.Arms, rightLowerArmSizeY, rightLowerArmDiameter/2.0, -massScale, 3);
  character.addArticulatedRigidBody(rLowerArm);

  joint:= createJoint(world, 'rElbow', TPoint3D.Create(0, -rightUpperArmSizeY/2.0, 0),
    TPoint3D.Create(0, rightLowerArmSizeY/2.0, 0), 0, 2.7, rUpperArm, rLowerArm);
  character.addJoint(joint);

  leftUpperLegSizeY:= self.getHipPosY() - self.getKneePosY(1);
  leftUpperLegDiameter:= self.getUpperLegDiameter(1);
  lUpperLeg:= createArticulatedCylinder(world, 'lUpperLeg', TBodyPart.Legs, leftUpperLegSizeY, leftUpperLegDiameter/2.0, -massScale, 4);
  character.addArticulatedRigidBody(lUpperLeg);

  joint:= createJoint(world, 'lHip', TPoint3D.Create(pelvisRadius*self.getLegRelativeAnchorX(1), -pelvisSizeY/2.0, 0),
    TPoint3D.Create(0, leftUpperLegSizeY/2.0, 0), -1.3, 1.9, pelvis, lUpperLeg);
  character.addJoint(joint);

  rightUpperLegSizeY:= self.getHipPosY() - self.getKneePosY(-1);
  rightUpperLegDiameter:= self.getUpperLegDiameter(-1);
  rUpperLeg:= createArticulatedCylinder(world, 'rUpperLeg', TBodyPart.Legs, rightUpperLegSizeY, rightUpperLegDiameter/2.0, -massScale, 4);
  character.addArticulatedRigidBody(rUpperLeg);

  joint:= createJoint(world, 'rHip', TPoint3D.Create(-pelvisRadius*self.getLegRelativeAnchorX(-1), -pelvisSizeY/2.0, 0),
    TPoint3D.Create(0, rightUpperLegSizeY/2.0, 0), -1.3, 1.9, pelvis, rUpperLeg);
  character.addJoint(joint);

  leftLowerLegSizeY:= self.getKneePosY(1) - self.getAnklePosY(1);
  leftLowerLegDiameter:= self.getLowerLegDiameter(1);
  lLowerLeg:= createArticulatedCylinder(world, 'lLowerLeg', TBodyPart.Legs, leftLowerLegSizeY, leftLowerLegDiameter/2.0, -massScale, 4);
  character.addArticulatedRigidBody(lLowerLeg);

  joint:= createJoint(world, 'lKnee', TPoint3D.Create(0, -leftUpperLegSizeY/2.0, 0),
    TPoint3D.Create(0, leftLowerLegSizeY/2.0, 0), -2.5, 0, lUpperLeg, lLowerLeg);
  character.addJoint(joint);

  rightLowerLegSizeY:= self.getKneePosY(-1) - self.getAnklePosY(-1);
  rightLowerLegDiameter:= self.getLowerLegDiameter(-1);
  rLowerLeg:= createArticulatedCylinder(world, 'rLowerLeg', TBodyPart.Legs, rightLowerLegSizeY, rightLowerLegDiameter/2.0, -massScale, 4);
  character.addArticulatedRigidBody(rLowerLeg);

  joint:= createJoint(world, 'rKnee', TPoint3D.Create(0, -rightUpperLegSizeY/2.0, 0),
    TPoint3D.Create(0, rightLowerLegSizeY/2.0, 0), -2.5, 0, rUpperLeg, rLowerLeg);
  character.addJoint(joint);

  leftFootSizeX:= self.getFootSizeX(1);
  leftFootSizeY:= self.getAnklePosY(1) - self.getGroundPosY();
  leftFootSizeZ:= self.getFootSizeZ(1) * 0.75;
  lFoot:= createArticulatedBox(world, 'lFoot', TBodyPart.Legs, leftFootSizeX,leftFootSizeY,leftFootSizeZ, -massScale, 3);
  character.addArticulatedRigidBody(lFoot);

  joint:= createJoint(world, 'lAnkle', TPoint3D.Create(0, -leftLowerLegSizeY/2.0, 0),
    TPoint3D.Create(0, leftFootSizeY/2.0, -leftFootSizeZ*0.33 + leftLowerLegDiameter/2.0), -0.75, 0.75, lLowerLeg, lFoot);
  character.addJoint(joint);

  rightFootSizeX:= self.getFootSizeX(-1);
  rightFootSizeY:= self.getAnklePosY(-1) - self.getGroundPosY();
  rightFootSizeZ:= self.getFootSizeZ(-1) * 0.75;
  rFoot:= createArticulatedBox(world, 'rFoot', TBodyPart.Legs, rightFootSizeX,rightFootSizeY,rightFootSizeZ, -massScale, 3);
  character.addArticulatedRigidBody(rFoot);

  joint:= createJoint(world, 'rAnkle', TPoint3D.Create(0, -rightLowerLegSizeY/2.0, 0),
    TPoint3D.Create(0, rightFootSizeY/2.0, -rightFootSizeZ*0.33 + rightLowerLegDiameter/2.0), -0.75, 0.75, rLowerLeg, rFoot);
  character.addJoint(joint);

  leftToesSizeX:= leftFootSizeX;
  leftToesSizeY:= leftFootSizeY * 0.66;
  leftToesSizeZ:= self.getFootSizeZ(1) - leftFootSizeZ;
  lToes:= createArticulatedBox(world, 'lToes', TBodyPart.Legs, leftToesSizeX,leftToesSizeY,leftToesSizeZ, -massScale, 3);
  character.addArticulatedRigidBody(lToes);

  joint:= createJoint(world, 'lToeJoint', TPoint3D.Create(0, (leftToesSizeY-leftFootSizeY)/2.0+0.003, leftFootSizeZ/2.0),
    TPoint3D.Create(0, 0, -leftToesSizeZ/2.0), -0.1, 0.52, lFoot, lToes);
  character.addJoint(joint);

  rightToesSizeX:= rightFootSizeX;
  rightToesSizeY:= rightFootSizeY * 0.66;
  rightToesSizeZ:= self.getFootSizeZ(-1) - rightFootSizeZ;
  rToes:= createArticulatedBox(world, 'rToes', TBodyPart.Legs, rightToesSizeX,rightToesSizeY,rightToesSizeZ, -massScale, 3);
  character.addArticulatedRigidBody(rToes);

  joint:= createJoint(world, 'rToeJoint', TPoint3D.Create(0, (rightToesSizeY-rightFootSizeY)/2.0+0.003, rightFootSizeZ/2.0),
    TPoint3D.Create(0, 0, -rightToesSizeZ/2.0), -0.1, 0.52, rFoot, rToes);
  character.addJoint(joint);

  character.completeFigure;

  Result:= character;
end;

function TCharacterDescription.createJoint(world: Tb2World; const Name: String; const posInParent, posInChild: TPoint3D; minAngle,
  maxAngle: Double; Parent, Child: TArticulatedRigidBody): TJoint;
var
  rjd: Tb2RevoluteJointDef;
  instance: Tb2RevoluteJoint;
begin
  rjd := Tb2RevoluteJointDef.Create;
  rjd.Initialize(Parent.Body, Child.Body, TVector2.From(0, 0));
  rjd.localAnchorA:= TVector2.From(posInParent.Z, posInParent.Y);
  rjd.localAnchorB:= TVector2.From(posInChild.Z, posInChild.Y);
  rjd.lowerAngle:= minAngle;
  rjd.upperAngle:= maxAngle;
  rjd.enableLimit:= True;
  rjd.enableMotor := False;
  instance:= World.CreateJoint(rjd) as Tb2RevoluteJoint;
  Result:= TJoint.Create(Name, instance);
end;

function TCharacterDescription.createArticulatedBox(world: Tb2World; const Name: String; BodyPart: TBodyPart; SizeX, SizeY, SizeZ, mass,
  moiScale: Double; PosX, PosY: Double): TArticulatedRigidBody;
var
  bd: Tb2BodyDef;
  fd: Tb2FixtureDef;
  shape: Tb2PolygonShape;
  instance: Tb2Body;
  MassData: Tb2MassData;
  volume: Double;
begin
  bd:= Tb2BodyDef.Create;
  bd.bodyType:= b2_dynamicBody;
  shape := Tb2PolygonShape.Create;
  shape.SetAsBox(SizeZ / 2, SizeY / 2); // 3D to 2D
  fd := Tb2FixtureDef.Create;
  fd.shape := shape;
  fd.density:= 1.0;
  fd.friction:= 0.8;
  fd.restitution:= 0.35;
  bd.position:= TVector2.From(PosX, PosY);
  instance := World.CreateBody(bd);
  instance.CreateFixture(fd);
  if mass < 0 then
  begin
    volume:= SizeX * SizeY * SizeZ;
    mass:= -mass * volume;
  end;
  MassData.mass:= mass;
  MassData.I:= (SizeZ * SizeZ + SizeY * SizeY) * (1/12) * mass * moiScale;
  MassData.center:= TVector2.From(0, 0);
  instance.SetMassData(MassData);

  Result:= TArticulatedRigidBody.Create(Name, BodyPart, instance, SizeZ, SizeY);
end;

function TCharacterDescription.createArticulatedEllipsoid(world: Tb2World; const Name: String; BodyPart: TBodyPart; RadiusX, RadiusY, RadiusZ, mass,
  moiScale: Double; PosX, PosY: Double): TArticulatedRigidBody;
var
  bd: Tb2BodyDef;
  fd: Tb2FixtureDef;
  shape: Tb2CircleShape;
  instance: Tb2Body;
  MassData: Tb2MassData;
  volume: Double;
begin
  bd:= Tb2BodyDef.Create;
  bd.bodyType:= b2_dynamicBody;
  shape := Tb2CircleShape.Create;
  shape.m_radius:= Min(RadiusZ, RadiusY);
  fd := Tb2FixtureDef.Create;
  fd.shape := shape;
  fd.density:= 1.0;
  fd.friction:= 0.8;
  fd.restitution:= 0.35;
  bd.position:= TVector2.From(PosX, PosY);
  instance := World.CreateBody(bd);
  instance.CreateFixture(fd);
  if mass < 0 then
  begin
    volume:= 4.0 / 3.0 * Pi * RadiusX * RadiusY * RadiusZ;
    mass:= -mass * volume;
  end;
  MassData.mass:= mass;
  MassData.I:= mass * (RadiusZ * RadiusZ + RadiusY * RadiusY) / 5.0 * moiScale;
  MassData.center:= TVector2.From(0, 0);
  instance.SetMassData(MassData);

  Result:= TArticulatedRigidBody.Create(Name, BodyPart, instance, shape.m_radius, shape.m_radius);
end;

function TCharacterDescription.createArticulatedCylinder(world: Tb2World; const Name: String; BodyPart: TBodyPart; Height, Radius, mass,
  moiScale: Double; PosX, PosY: Double): TArticulatedRigidBody;
var
  bd: Tb2BodyDef;
  fd: Tb2FixtureDef;
  shape: Tb2PolygonShape;
  instance: Tb2Body;
  MassData: Tb2MassData;
  volume: Double;
begin
  bd:= Tb2BodyDef.Create;
  bd.bodyType:= b2_dynamicBody;
  shape := Tb2PolygonShape.Create;
  shape.SetAsBox(Radius, Height / 2); // 3D to 2D
  fd := Tb2FixtureDef.Create;
  fd.shape := shape;
  fd.density:= 1.0;
  fd.friction:= 0.8;
  fd.restitution:= 0.35;
  bd.position:= TVector2.From(PosX, PosY);
  instance := World.CreateBody(bd);
  instance.CreateFixture(fd);
  if mass < 0 then
  begin
    volume:= Pi * Radius * Radius * Height;
    mass:= -mass * volume;
  end;
  MassData.mass:= mass;
  MassData.I:= mass * (3 * Radius * Radius + Height * Height) / 12.0;
  MassData.center:= TVector2.From(0, 0);
  instance.SetMassData(MassData);

  Result:= TArticulatedRigidBody.Create(Name, BodyPart, instance, Radius * 2, Height);
end;

{$REGION 'GENERATED'}

// native variables

function TCharacterDescription.GetFootSizeX(side: Integer) : Double;
begin
  Result:= _footSizeX[side];
end;

procedure TCharacterDescription.SetFootSizeX(side: Integer; value: Double);
begin
  if SameValue(_footSizeX[side], value) then Exit;
  if self._isSymmetric then side:= 0;
  self._indirectVarsAreValid:= False;
  _footSizeX[side]:= value;
end;

function TCharacterDescription.GetFootSizeZ(side: Integer) : Double;
begin
  Result:= _footSizeZ[side];
end;

procedure TCharacterDescription.SetFootSizeZ(side: Integer; value: Double);
begin
  if SameValue(_footSizeZ[side], value) then Exit;
  if self._isSymmetric then side:= 0;
  self._indirectVarsAreValid:= False;
  _footSizeZ[side]:= value;
end;

function TCharacterDescription.GetAnkleRelativePosY(side: Integer) : Double;
begin
  Result:= _ankleRelativePosY[side];
end;

procedure TCharacterDescription.SetAnkleRelativePosY(side: Integer; value: Double);
begin
  if SameValue(_ankleRelativePosY[side], value) then Exit;
  if self._isSymmetric then side:= 0;
  self._indirectVarsAreValid:= False;
  _ankleRelativePosY[side]:= value;
end;

function TCharacterDescription.GetLowerLegDiameter(side: Integer) : Double;
begin
  Result:= _lowerLegDiameter[side];
end;

procedure TCharacterDescription.SetLowerLegDiameter(side: Integer; value: Double);
begin
  if SameValue(_lowerLegDiameter[side], value) then Exit;
  if self._isSymmetric then side:= 0;
  self._indirectVarsAreValid:= False;
  _lowerLegDiameter[side]:= value;
end;

function TCharacterDescription.GetUpperLegDiameter(side: Integer) : Double;
begin
  Result:= _upperLegDiameter[side];
end;

procedure TCharacterDescription.SetUpperLegDiameter(side: Integer; value: Double);
begin
  if SameValue(_upperLegDiameter[side], value) then Exit;
  if self._isSymmetric then side:= 0;
  self._indirectVarsAreValid:= False;
  _upperLegDiameter[side]:= value;
end;

function TCharacterDescription.GetLegSizeY : Double;
begin
  Result:= _legSizeY;
end;

procedure TCharacterDescription.SetLegSizeY(value: Double);
begin
  if SameValue(_legSizeY.value, value) then Exit;
  self._indirectVarsAreValid:= False;
  _legSizeY.value:= value;
end;

function TCharacterDescription.GetKneeRelativePosY(side: Integer) : Double;
begin
  Result:= _kneeRelativePosY[side];
end;

procedure TCharacterDescription.SetKneeRelativePosY(side: Integer; value: Double);
begin
  if SameValue(_kneeRelativePosY[side], value) then Exit;
  if self._isSymmetric then side:= 0;
  self._indirectVarsAreValid:= False;
  _kneeRelativePosY[side]:= value;
end;

function TCharacterDescription.GetLegRelativeAnchorX(side: Integer) : Double;
begin
  Result:= _legRelativeAnchorX[side];
end;

procedure TCharacterDescription.SetLegRelativeAnchorX(side: Integer; value: Double);
begin
  if SameValue(_legRelativeAnchorX[side], value) then Exit;
  if self._isSymmetric then side:= 0;
  self._indirectVarsAreValid:= False;
  _legRelativeAnchorX[side]:= value;
end;

function TCharacterDescription.GetPelvisDiameter : Double;
begin
  Result:= _pelvisDiameter;
end;

procedure TCharacterDescription.SetPelvisDiameter(value: Double);
begin
  if SameValue(_pelvisDiameter.value, value) then Exit;
  self._indirectVarsAreValid:= False;
  _pelvisDiameter.value:= value;
end;

function TCharacterDescription.GetTorsoDiameter : Double;
begin
  Result:= _torsoDiameter;
end;

procedure TCharacterDescription.SetTorsoDiameter(value: Double);
begin
  if SameValue(_torsoDiameter.value, value) then Exit;
  self._indirectVarsAreValid:= False;
  _torsoDiameter.value:= value;
end;

function TCharacterDescription.GetTrunkSizeY : Double;
begin
  Result:= _trunkSizeY;
end;

procedure TCharacterDescription.SetTrunkSizeY(value: Double);
begin
  if SameValue(_trunkSizeY.value, value) then Exit;
  self._indirectVarsAreValid:= False;
  _trunkSizeY.value:= value;
end;

function TCharacterDescription.GetWaistRelativePosY : Double;
begin
  Result:= _waistRelativePosY;
end;

procedure TCharacterDescription.SetWaistRelativePosY(value: Double);
begin
  if SameValue(_waistRelativePosY.value, value) then Exit;
  self._indirectVarsAreValid:= False;
  _waistRelativePosY.value:= value;
end;

function TCharacterDescription.GetChestRelativePosY : Double;
begin
  Result:= _chestRelativePosY;
end;

procedure TCharacterDescription.SetChestRelativePosY(value: Double);
begin
  if SameValue(_chestRelativePosY.value, value) then Exit;
  self._indirectVarsAreValid:= False;
  _chestRelativePosY.value:= value;
end;

function TCharacterDescription.GetNeckSizeY : Double;
begin
  Result:= _neckSizeY;
end;

procedure TCharacterDescription.SetNeckSizeY(value: Double);
begin
  if SameValue(_neckSizeY.value, value) then Exit;
  self._indirectVarsAreValid:= False;
  _neckSizeY.value:= value;
end;

function TCharacterDescription.GetHeadSizeX : Double;
begin
  Result:= _headSizeX;
end;

procedure TCharacterDescription.SetHeadSizeX(value: Double);
begin
  if SameValue(_headSizeX.value, value) then Exit;
  self._indirectVarsAreValid:= False;
  _headSizeX.value:= value;
end;

function TCharacterDescription.GetHeadSizeY : Double;
begin
  Result:= _headSizeY;
end;

procedure TCharacterDescription.SetHeadSizeY(value: Double);
begin
  if SameValue(_headSizeY.value, value) then Exit;
  self._indirectVarsAreValid:= False;
  _headSizeY.value:= value;
end;

function TCharacterDescription.GetHeadSizeZ : Double;
begin
  Result:= _headSizeZ;
end;

procedure TCharacterDescription.SetHeadSizeZ(value: Double);
begin
  if SameValue(_headSizeZ.value, value) then Exit;
  self._indirectVarsAreValid:= False;
  _headSizeZ.value:= value;
end;

function TCharacterDescription.GetUpperArmDiameter(side: Integer) : Double;
begin
  Result:= _upperArmDiameter[side];
end;

procedure TCharacterDescription.SetUpperArmDiameter(side: Integer; value: Double);
begin
  if SameValue(_upperArmDiameter[side], value) then Exit;
  if self._isSymmetric then side:= 0;
  self._indirectVarsAreValid:= False;
  _upperArmDiameter[side]:= value;
end;

function TCharacterDescription.GetLowerArmDiameter(side: Integer) : Double;
begin
  Result:= _lowerArmDiameter[side];
end;

procedure TCharacterDescription.SetLowerArmDiameter(side: Integer; value: Double);
begin
  if SameValue(_lowerArmDiameter[side], value) then Exit;
  if self._isSymmetric then side:= 0;
  self._indirectVarsAreValid:= False;
  _lowerArmDiameter[side]:= value;
end;

function TCharacterDescription.GetArmSizeY(side: Integer) : Double;
begin
  Result:= _armSizeY[side];
end;

procedure TCharacterDescription.SetArmSizeY(side: Integer; value: Double);
begin
  if SameValue(_armSizeY[side], value) then Exit;
  if self._isSymmetric then side:= 0;
  self._indirectVarsAreValid:= False;
  _armSizeY[side]:= value;
end;

function TCharacterDescription.GetElbowRelativePosY(side: Integer) : Double;
begin
  Result:= _elbowRelativePosY[side];
end;

procedure TCharacterDescription.SetElbowRelativePosY(side: Integer; value: Double);
begin
  if SameValue(_elbowRelativePosY[side], value) then Exit;
  if self._isSymmetric then side:= 0;
  self._indirectVarsAreValid:= False;
  _elbowRelativePosY[side]:= value;
end;

function TCharacterDescription.GetGroundPosY : Double;
begin
  Result:= _groundPosY;
end;

procedure TCharacterDescription.SetGroundPosY(value: Double);
begin
  if SameValue(_groundPosY.value, value) then Exit;
  self._indirectVarsAreValid:= False;
  _groundPosY.value:= value;
end;

// indirect variables

function TCharacterDescription.GetLegPosX(side: Integer) : Double;
begin
  if not self._indirectVarsAreValid then self.computeIndirectVars();
  Result:= _legPosX[side];
end;

procedure TCharacterDescription.ComputeLegPosX(side: Integer);
begin
  self._legPosX.setSide( side, side*self._pelvisDiameter.get()/2.0*self._legRelativeAnchorX.getSide(side))
end;

procedure TCharacterDescription.SetLegPosX(side: Integer; value: Double);
begin
  self.setLegRelativeAnchorX( side, value/self._pelvisDiameter.get()*2.0*side )
end;

function TCharacterDescription.GetAnklePosY(side: Integer) : Double;
begin
  if not self._indirectVarsAreValid then self.computeIndirectVars();
  Result:= _anklePosY[side];
end;

procedure TCharacterDescription.ComputeAnklePosY(side: Integer);
begin
  self._anklePosY.setSide( side, self._groundPosY.get() + self._legSizeY.get() * self._ankleRelativePosY.getSide(side) )
end;

procedure TCharacterDescription.SetAnklePosY(side: Integer; value: Double);
begin
  self.setAnkleRelativePosY( side, (value - self._groundPosY.get())/self._legSizeY.get() )
end;

function TCharacterDescription.GetKneePosY(side: Integer) : Double;
begin
  if not self._indirectVarsAreValid then self.computeIndirectVars();
  Result:= _kneePosY[side];
end;

procedure TCharacterDescription.ComputeKneePosY(side: Integer);
begin
  self._kneePosY.setSide( side, self._anklePosY.getSide(side) + (self._hipPosY.get()-self._anklePosY.getSide(side)) * self._kneeRelativePosY.getSide(side) )
end;

procedure TCharacterDescription.SetKneePosY(side: Integer; value: Double);
begin
  self.setKneeRelativePosY( side, (value - self._anklePosY.getSide(side)) / (self._hipPosY.get()-self._anklePosY.getSide(side)) )
end;

function TCharacterDescription.GetHipPosY : Double;
begin
  if not self._indirectVarsAreValid then self.computeIndirectVars();
  Result:= _hipPosY;
end;

procedure TCharacterDescription.ComputeHipPosY;
begin
  self._hipPosY.setValue( self._groundPosY.get() + self._legSizeY.get() )
end;

procedure TCharacterDescription.SetHipPosY(value: Double);
begin
  self.setLegSizeY( value - self._groundPosY.get() )
end;

function TCharacterDescription.GetWaistPosY : Double;
begin
  if not self._indirectVarsAreValid then self.computeIndirectVars();
  Result:= _waistPosY;
end;

procedure TCharacterDescription.ComputeWaistPosY;
begin
  self._waistPosY.setValue( self._hipPosY.get() + self._trunkSizeY.get() * self._waistRelativePosY.get() )
end;

procedure TCharacterDescription.SetWaistPosY(value: Double);
begin
  self.setWaistRelativePosY( (value - self._hipPosY.get())/self._trunkSizeY.get() )
end;

function TCharacterDescription.GetChestPosY : Double;
begin
  if not self._indirectVarsAreValid then self.computeIndirectVars();
  Result:= _chestPosY;
end;

procedure TCharacterDescription.ComputeChestPosY;
begin
  self._chestPosY.setValue( self._waistPosY.get() + (self._shoulderPosY.get()-self._waistPosY.get()) * self._chestRelativePosY.get() )
end;

procedure TCharacterDescription.SetChestPosY(value: Double);
begin
  self.setChestRelativePosY( (value - self._waistPosY.get())/(self._shoulderPosY.get()-self._waistPosY.get()) )
end;

function TCharacterDescription.GetShoulderPosY : Double;
begin
  if not self._indirectVarsAreValid then self.computeIndirectVars();
  Result:= _shoulderPosY;
end;

procedure TCharacterDescription.ComputeShoulderPosY;
begin
  self._shoulderPosY.setValue( self._hipPosY.get() + self._trunkSizeY.get() )
end;

procedure TCharacterDescription.SetShoulderPosY(value: Double);
begin
  self.setTrunkSizeY( value - self._hipPosY.get() )
end;

function TCharacterDescription.GetNeckPosY : Double;
begin
  if not self._indirectVarsAreValid then self.computeIndirectVars();
  Result:= _neckPosY;
end;

procedure TCharacterDescription.ComputeNeckPosY;
begin
  self._neckPosY.setValue( self._shoulderPosY.get() + self._neckSizeY.get() )
end;

procedure TCharacterDescription.SetNeckPosY(value: Double);
begin
  self.setNeckSizeY( value - self._shoulderPosY.get() )
end;

function TCharacterDescription.GetArmPosX(side: Integer) : Double;
begin
  if not self._indirectVarsAreValid then self.computeIndirectVars();
  Result:= _armPosX[side];
end;

procedure TCharacterDescription.ComputeArmPosX(side: Integer);
begin
  self._armPosX.setSide(side, side*(self._torsoDiameter.get() + self._upperArmDiameter.getSide(side))/2.0)
end;

function TCharacterDescription.GetElbowPosY(side: Integer) : Double;
begin
  if not self._indirectVarsAreValid then self.computeIndirectVars();
  Result:= _elbowPosY[side];
end;

procedure TCharacterDescription.ComputeElbowPosY(side: Integer);
begin
  self._elbowPosY.setSide(side, self._shoulderPosY.get() - self._armSizeY.getSide(side)*self._elbowRelativePosY.getSide(side))
end;

procedure TCharacterDescription.SetElbowPosY(side: Integer; value: Double);
begin
  self.setElbowRelativePosY( side, (self._shoulderPosY.get() - value)/self._armSizeY.getSide(side) )
end;

function TCharacterDescription.GetWristPosY(side: Integer) : Double;
begin
  if not self._indirectVarsAreValid then self.computeIndirectVars();
  Result:= _wristPosY[side];
end;

procedure TCharacterDescription.ComputeWristPosY(side: Integer);
begin
  self._wristPosY.setSide(side, self._shoulderPosY.get() - self._armSizeY.getSide(side))
end;

procedure TCharacterDescription.SetWristPosY(side: Integer; value: Double);
begin
  self.setArmSizeY( side, self._shoulderPosY.get() - value )
end;

{$ENDREGION}

{ TMinMax }

constructor TMinMax.Create(minValue, maxValue: Double);
begin
  if not IsNan(maxValue) and not IsNan(minValue) and (maxValue < minValue) then
    raise EInvalidArgument.Create('Max must be equal or greater than minValue!');
  self._minValue:= minValue;
  self._maxValue:= maxValue;
end;

function TMinMax.clamp(value: Double): Double;
begin
  if not IsNan(self._minValue) and (value < self._minValue) then
    Exit(self._minValue);
  if not IsNan(self._maxValue) and (value > self._maxValue) then
    Exit(self._maxValue);
  Result:= value;
end;

{ TValue }

constructor TValue.Create(value, minValue, maxValue: Double);
begin
  self.MinMax:= TMinMax.Create(minValue, maxValue);
  self._value:= value;
end;

procedure TValue.SetValue(Value: Double);
begin
  _value:= MinMax.clamp(Value);
end;

function TValue.get: Double;
begin
  Result:= value;
end;

class operator TValue.Implicit(const Value: TValue): Double;
begin
  Result:= Value.value;
end;

{ TSymmetric }

constructor TSymmetric.Create(value, minValue, maxValue: Double);
begin
  self.MinMax:= TMinMax.Create(minValue, maxValue);
  self._side[0]:= value;
  self._side[1]:= value;
end;

function TSymmetric.GetSide(side: Integer): Double;
begin
  Result:= self._side[(side+1) shr 1];
end;

procedure TSymmetric.SetSide(side: Integer; value: Double);
begin
  value:= MinMax.clamp(value);
  if side = 0 then
  begin
    self._side[0]:= value;
    self._side[1]:= value;
    Exit;
  end;

  if (side <> -1) and (side <> 1) then
    raise EInvalidArgument.Create('Side must be -1, 0 or 1.');
  self._side[(side+1) shr 1]:= value;
end;

procedure TSymmetric.SetRight(value: Double);
begin
  self._side[0]:= MinMax.clamp(value);
end;

procedure TSymmetric.SetLeft(value: Double);
begin
  self._side[1]:= MinMax.clamp(value);
end;

procedure TSymmetric.forceSymmetric;
begin
  self._side[0]:= self._side[1];
end;

end.

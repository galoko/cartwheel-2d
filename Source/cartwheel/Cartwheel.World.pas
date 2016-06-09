unit Cartwheel.World;

interface

uses
  System.Math, System.Generics.Collections,
  Box2D.Physics2D, Box2D.Physics2DTypes,
  Cartwheel.Character, Cartwheel.Controllers;

type
  TCartwheelWorld = class
  strict private
    World: Tb2World;
    AFs: TList<TArticulatedFigure>;
    jts: TList<TJoint>;
    controllerList: TList<TController>;
  public
    Enabled: Boolean;
    constructor Create(World: Tb2World);
    procedure addArticulatedFigure(articulatedFigure: TArticulatedFigure);
    procedure addController(controller: TController);
    procedure removeController(controller: TController);
    procedure addContact(obj1, obj2: TObject; const contact: Tb2Contact; const impulse: Tb2ContactImpulse; dt: Double);
    procedure performPreTasks(dt: Double);
    procedure advanceInTime(deltaT: Double);
    procedure performPostTasks(dt: Double);
    class function shouldCheckForCollisions(rb1, rb2: TArticulatedRigidBody) : Boolean; static;
  end;

implementation

{ TCartwheelWorld }

constructor TCartwheelWorld.Create(World: Tb2World);
begin
  Enabled:= True;
  Self.World:= World;
  AFs:= TList<TArticulatedFigure>.Create;
  jts:= TList<TJoint>.Create;
  controllerList:= TList<TController>.Create;
end;

procedure TCartwheelWorld.addArticulatedFigure(articulatedFigure: TArticulatedFigure);
begin
  AFs.Add(articulatedFigure);
  articulatedFigure.addJointsToList(jts);
end;

procedure TCartwheelWorld.addController(controller: TController);
begin
  controllerList.Add(controller);
end;

procedure TCartwheelWorld.removeController(controller: TController);
begin
  controllerList.Remove(controller);
end;

procedure TCartwheelWorld.addContact(obj1, obj2: TObject; const contact: Tb2Contact; const impulse: Tb2ContactImpulse; dt: Double);
var
  rb1, rb2: TArticulatedRigidBody;
  worldManifold: Tb2WorldManifold;
  Index: Integer;
  ContactPoint: TContactPoint;
begin
  if not Enabled then Exit;

  if obj1 is TArticulatedRigidBody then
    rb1:= TArticulatedRigidBody(obj1)
  else rb1:= nil;
  if obj2 is TArticulatedRigidBody then
    rb2:= TArticulatedRigidBody(obj2)
  else rb2:= nil;

  // нет тел - никому не нужна точка - выходим без лишних вычислений
  if (rb1 = nil) and (rb2 = nil) then Exit;

  contact.GetWorldManifold(worldManifold);

  for Index := 0 to impulse.count - 1 do
  begin
    ContactPoint.cp:= worldManifold.points[Index];
    ContactPoint.rb1:= rb1;
    ContactPoint.rb2:= rb2;
    // очень надеюсь что я правильно понял и это сила примененная к fixtureA
    ContactPoint.f:= -(worldManifold.normal * impulse.normalImpulses[Index]) / dt;

    if Assigned(rb1) then
      rb1.AFParent.addContact(ContactPoint);

    if Assigned(rb2) then
      rb2.AFParent.addContact(ContactPoint);
  end;
end;

procedure TCartwheelWorld.performPreTasks(dt: Double);
var
  controller: TController;
begin
  if not Enabled then Exit;

  for controller in controllerList do
    controller.performPreTasks(dt, controller.character.Contacts);
end;

procedure TCartwheelWorld.advanceInTime(deltaT: Double);
var
  j: TJoint;
  t: Double;
  AF: TArticulatedFigure;
begin
  if not Enabled then Exit;

  if deltaT <= 0 then Exit;

	for j in jts do
  begin
		t:= j.torque;
		//we will apply to the parent a positive torque, and to the child a negative torque
    j.parent.Body.ApplyTorque(t);
    j.child.Body.ApplyTorque(-t);
	end;

  for AF in AFs do
    AF.clearContacts;
end;

procedure TCartwheelWorld.performPostTasks(dt: Double);
var
  controller: TController;
begin
  if not Enabled then Exit;

  for controller in controllerList do
    controller.performPostTasks(dt, controller.character.Contacts);
end;

class function TCartwheelWorld.shouldCheckForCollisions(rb1, rb2: TArticulatedRigidBody): Boolean;

  function HaveBP(BP: TBodyPart) : Boolean;
  begin
    Result:= (rb1.BodyPart = BP) or (rb2.BodyPart = BP);
  end;

begin
  // разные фигуры должны сталкиваться
  if rb1.AFParent <> rb2.AFParent then Exit(True);

  // одинаковые части тела не сталкиваются (кроме тела и головы)
  if rb1.BodyPart = rb2.BodyPart then
  begin
    if rb1.BodyPart = TBodyPart.TorsoAndHead then
      Exit(True)
    else
      Exit(False);
  end;

  // тело и ноги сталкиваются
  if HaveBP(TBodyPart.TorsoAndHead) and HaveBP(TBodyPart.Legs) then Exit(True);

  Result:= False;
end;

function CartwheelMixFriction(friction1, friction2: PhysicsFloat): PhysicsFloat;
begin
  Result:= Min(friction1, friction2);
end;

function CartwheelMixRestitution(restitution1, restitution2: PhysicsFloat): PhysicsFloat;
begin
  Result:= Min(restitution1, restitution2);
end;

begin
  b2MixFriction:= CartwheelMixFriction;
  b2MixRestitution:= CartwheelMixRestitution;
end.

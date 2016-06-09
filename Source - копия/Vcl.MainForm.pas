unit Vcl.MainForm;

{$SCOPEDENUMS ON}

interface

uses
  Winapi.Windows,
  Box2D.Physics2D, Box2D.Physics2DTypes,
  System.SysUtils, System.Types, System.UITypes, System.Classes, System.Variants, System.Math, System.Math.Vectors,
  FMX.Types, FMX.Controls, FMX.Forms, FMX.Dialogs, FMX.Ani, FMX.Layouts, FMX.Gestures, FMX.Objects, FMX.Graphics, FMX.StdCtrls,
  Cartwheel.CharacterDescription, Cartwheel.Character, Cartwheel.World, Cartwheel.Controllers, Cartwheel.WorldOracle, Cartwheel.EditableWalk,
  Cartwheel.TwoLinkIK;

const
  GRAVITY = -9.8;

  Step = 1 / 100;
  OneStep = 0.0005;

type
  TMainForm = class(TForm)
    Image: TImage;
    procedure ImageMouseDown(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Single);
    procedure ImageMouseMove(Sender: TObject; Shift: TShiftState; X, Y: Single);
    procedure ImageMouseUp(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Single);
    procedure FormCreate(Sender: TObject);
    procedure Idle(Sender: TObject; var Done: Boolean);
    procedure ImageMouseWheel(Sender: TObject; Shift: TShiftState; WheelDelta: Integer; var Handled: Boolean);
    procedure FormKeyDown(Sender: TObject; var Key: Word; var KeyChar: Char; Shift: TShiftState);
    procedure FormKeyUp(Sender: TObject; var Key: Word; var KeyChar: Char; Shift: TShiftState);
  private
    { Private declarations }
  public
    { Public declarations }

    // timing
    LastStepTime: Double;

    // vcl
    LastMousePos, MousePos: TPointF;
    MouseStates: array [TMouseButton] of Boolean;

    // box2d
    World: Tb2World;
    Floor: Tb2Body;

    // cartwheel
    CartwheelWorld: TCartwheelWorld;
    character: TCharacter;
    controller: TEditableWalking;

    // controls
    VkRightIsPressed, VkLeftIsPressed: Boolean;
    PushTime, PushForce: PhysicsFloat;

    // scene
    Scene: TCanvas;
    SceneZero: TVector2;
    Scale: PhysicsFloat;
    TimeScale: PhysicsFloat;
    Pause: Boolean;

    // mouse
    MouseJoint: Tb2MouseJoint;
    MouseBody: Tb2Body;

    // debug
    HipAngle, KneeAngle: Double;

    procedure CreateFloor;
    procedure CreateBox;
    procedure CreateCharacter;

    procedure ApplyPushForce(Dt: PhysicsFloat);

    procedure SetupCamera;

    // paint
    procedure BeginPaint;
    procedure DrawDebugVector(const V: TVector2; Color: TAlphaColor; Width: PhysicsFloat = 35);
    procedure DrawDebugText(const V: TVector2; const Format: String; const Args: array of const; RightToLeft: Boolean = False); overload;
    procedure DrawDebugText(const V: TVector2; const Text: String; RightToLeft: Boolean = False); overload;
    procedure EndPaint;

    // vcl
    procedure UpdateMouse;
    procedure CreateMouseJoint;

    // coordinate convert utils
    class function ToScene(V: TVector2) : TPointF; static;
    class function ToBox2D(P: TPointF) : TVector2; static;
  end;

  TDestructionListener = class(Tb2DestructionListener)
  public
    procedure SayGoodbye(fixture: Tb2Fixture); overload; override;
    procedure SayGoodbye(joint: Tb2Joint); overload; override;
  end;

  TContactFilter = class(Tb2ContactFilter)
  public
    function ShouldCollide(fixtureA, fixtureB: Tb2Fixture): Boolean; override;
  end;

  TContactListener = class(Tb2ContactListener)
   public
     procedure BeginContact(var contact: Tb2Contact); override;
     procedure EndContact(var contact: Tb2Contact); override;
     procedure PreSolve(var contact: Tb2Contact; const oldManifold: Tb2Manifold); override;
     procedure PostSolve(var contact: Tb2Contact; const impulse: Tb2ContactImpulse); override;
   end;

   TDraw = class(Tb2Draw)
   public
     constructor Create;
     procedure DrawSolidPolygon(const vertices: Tb2PolyVertices; vertexCount: Int32; const color: RGBA); override;
     procedure DrawSegment(const p1, p2: TVector2; const color: RGBA); override;
     procedure DrawTransform(const xf: Tb2Transform); override;
     procedure DrawSolidCircle(const center, axis: TVector2; radius: PhysicsFloat; const color: RGBA); override;
   end;

var
  MainForm: TMainForm;

function GetRawReferenceTime: Double;

implementation

const
  EmptyPoint : TPointF = (X: -1; Y: -1);

{$R *.fmx}

procedure TMainForm.FormCreate(Sender: TObject);
begin
  TimeScale:= 1;

  LastMousePos:= EmptyPoint;

  Image.Bitmap.SetSize(1280, 960);
  Scene:= Image.Bitmap.Canvas;
  Scale:= 300;
  SceneZero:= TVector2.From(Scene.Width / 2, -Scene.Height / 2 * 1.4) / Scale;

  World:= Tb2World.Create(TVector2.From(0, GRAVITY));
  World.DestructionListener:= TDestructionListener.Create;
  World.SetContactFilter(TContactFilter.Create);
  World.SetContactListener(TContactListener.Create);
  World.Draw:= TDraw.Create;
  World.WarmStarting:= True;
  World.ContinuousPhysics:= True; // TOI

	MouseBody:= World.CreateBody(Tb2BodyDef.Create);

  CartwheelWorld:= TCartwheelWorld.Create(World);

  CreateFloor;
  CreateBox;
  CreateCharacter;

  Pause:= False;

  LastStepTime:= GetRawReferenceTime;
  Application.OnIdle:= Idle;
end;

procedure TMainForm.FormKeyDown(Sender: TObject; var Key: Word; var KeyChar: Char; Shift: TShiftState);
begin
  if Key = 0 then
  begin
    Key:= VkKeyScan(KeyChar);
    KeyChar:= #0;
  end;

  case AnsiChar(Key) of
  'P': Pause:= not Pause;
  AnsiChar(VK_RIGHT): VkRightIsPressed:= True;
  AnsiChar(VK_LEFT): VkLeftIsPressed:= True;
  AnsiChar(443): TimeScale:= Max(1, TimeScale / 2);
  AnsiChar(189): TimeScale:= Min(TimeScale * 2, 16);
  end;
end;

procedure TMainForm.FormKeyUp(Sender: TObject; var Key: Word; var KeyChar: Char; Shift: TShiftState);
begin
  if Key = 0 then
  begin
    Key:= VkKeyScan(KeyChar);
    KeyChar:= #0;
  end;
  case Key of
  VK_RIGHT: VkRightIsPressed:= False;
  VK_LEFT: VkLeftIsPressed:= False;
  end;
end;

procedure TMainForm.CreateFloor;
var
  bd: Tb2BodyDef;
  fd: Tb2FixtureDef;
  edge: Tb2EdgeShape;
begin
  bd:= Tb2BodyDef.Create;
  edge:= Tb2EdgeShape.Create;
  edge.SetVertices(MakeVector(-3.0, 0.0), MakeVector(3.0, 0.0));
  fd:= Tb2FixtureDef.Create;
  fd.shape:= edge;
  fd.friction:= 1.0;
  Floor:= World.CreateBody(bd);
  Floor.CreateFixture(fd);

  bd:= Tb2BodyDef.Create;
  edge:= Tb2EdgeShape.Create;
  edge.SetVertices(MakeVector(-3.0, 0.05), MakeVector(3.0, 0.05));
  fd:= Tb2FixtureDef.Create;
  fd.shape:= edge;
  fd.friction:= 1.0;
  bd.position:= TVector2.From(6.0, 0.0);
  Floor:= World.CreateBody(bd);
  Floor.CreateFixture(fd);
end;

procedure TMainForm.CreateBox;
const
  Radius = 1;
  PosX = 3.0;
  PosY = 1;
var
  bd: Tb2BodyDef;
  fd: Tb2FixtureDef;
  shape: Tb2PolygonShape;
  instance: Tb2Body;
  MassData: Tb2MassData;
begin
  bd:= Tb2BodyDef.Create;
  bd.bodyType:= b2_dynamicBody;
  shape := Tb2PolygonShape.Create;
  shape.SetAsBox(Radius, Radius / 100);
  fd := Tb2FixtureDef.Create;
  fd.shape := shape;
  fd.density:= 1.0;
  fd.friction:= 1.8;
  fd.restitution:= 0.35;
  bd.position:= TVector2.From(PosX, PosY);
  instance := World.CreateBody(bd);
  instance.CreateFixture(fd);
  MassData.mass:= 20;
  MassData.I:= 0.2;
  MassData.center:= TVector2.From(0, 0);
  instance.SetMassData(MassData);
  instance.SetLinearVelocity(TVector2.From(-0.0, -0.0));
end;


procedure TMainForm.CreateCharacter;
var
  desc: TCharacterDescription;
  worldOracle: TWorldOracle;
  behaviour: TBehaviourController;
begin
  desc:= TCharacterDescription.Create;

  character:= desc.createCharacter(World, TVector2.From(0, 0));
  CartwheelWorld.addArticulatedFigure(character);

  desc.Free;

  controller:= TEditableWalking.Create(character);
  CartwheelWorld.addController(controller);

  worldOracle:= TWorldOracle.Create;
  worldOracle.initializeWorld(world);

  behaviour:= TBehaviourController.Create(character, controller, worldOracle);
  behaviour.initializeDefaultParameters;

  behaviour.requestVelocities(1.0);
  behaviour.requestStepTime(0.4);

  controller.setBehaviour(behaviour);

  behaviour.conTransitionPlan;
end;

function PointToRect(const P: TPointF; Width: PhysicsFloat) : TRectF;
begin
  Result:= TRectF.Create(P, Width, Width);
  Result.Offset(-Width / 2, -Width / 2);
end;

function PointToVec2(const P: TPointF) : TVector2;
begin
  Result:= TVector2.From(P.X, P.Y);
end;

function Vec2ToPoint(const V: TVector2) : TPointF;
begin
  Result:= TPointF.Create(V.X, V.Y);
end;

procedure TMainForm.ApplyPushForce(Dt: PhysicsFloat);
var
  ImpulsePoint, Impulse: TVector2;
begin
  if VkRightIsPressed then
  begin
    PushTime:= 0.2;
    PushForce:= 100;
  end
  else
  if VkLeftIsPressed then
  begin
    PushTime:= 0.2;
    PushForce:= -100;
  end;

	if PushTime>0.0 then
  begin
		PushTime:= PushTime - Dt;     // subtract from duration
    Impulse:= TVector2.From(PushForce, 0);

    DrawDebugVector(ImpulsePoint, TAlphaColorRec.Brown, 14);
  end;
end;

procedure TMainForm.SetupCamera;
begin
  // SceneZero.X:= ClientWidth / Scale / 2 - controller.comPosition.x;
end;

{$DEFINE DRAW_DEBUG}

procedure TMainForm.DrawDebugText(const V: TVector2; const Format: String; const Args: array of const; RightToLeft: Boolean);
begin
  DrawDebugText(V, System.SysUtils.Format(Format, Args), RightToLeft);
end;

procedure TMainForm.DrawDebugText(const V: TVector2; const Text: String; RightToLeft: Boolean);
var
  Flags: TFillTextFlags;
  ATextAlign: TTextAlign;
  Rect: TRectF;
begin
  {$IFDEF DRAW_DEBUG}
  Scene.Fill.Color:= TAlphaColorRec.White;
  Scene.Font.Size:= 16;
  if RightToLeft then
  begin
    Flags:= [TFillTextFlag.RightToLeft];
    ATextAlign:= TTextAlign.Leading;
    Rect:= TRectF.Create(ToScene(V), -500, -35);
    Rect.NormalizeRect;
  end
  else
  begin
    Flags:= [];
    ATextAlign:= TTextAlign.Leading;
    Rect:= TRectF.Create(ToScene(V), 500, 35);
  end;
  Scene.FillText(Rect, Text, True, 1, Flags, ATextAlign, TTextAlign.Center);
  {$ENDIF}
end;

procedure TMainForm.DrawDebugVector(const V: TVector2; Color: TAlphaColor; Width: PhysicsFloat);
begin
  {$IFDEF DRAW_DEBUG}
  Scene.StrokeThickness:= 2.0;
  Scene.Stroke.Color:= Color;
  Scene.DrawEllipse(PointToRect(ToScene(V), Width * (Scale / 450)), 1);
  {$ENDIF}
end;

procedure TMainForm.Idle(Sender: TObject; var Done: Boolean);
var
  StepCounter, Count: Integer;
begin
  Done:= False;
  if GetRawReferenceTime - LastStepTime < Step then Exit;
  LastStepTime:= LastStepTime + Step;
  BeginPaint;

  SetupCamera;

  Count:= Trunc(Step / OneStep / TimeScale);

  if not Pause then
    for StepCounter := Count downto 1 do
    begin
      ApplyPushForce(OneStep); // pushes logic

      CartwheelWorld.performPreTasks(OneStep);

      CartwheelWorld.advanceInTime(OneStep);

      World.Step(OneStep, 1, 1, False);

      CartwheelWorld.performPostTasks(OneStep);
    end;

  DrawDebugText(TVector2.From(0, 0), '%f', [RadToDeg(HipAngle)]);
  DrawDebugText(TVector2.From(0, -0.2), '%f', [RadToDeg(KneeAngle)]);

  // DrawDebugVector(controller.Debug_pNow, TAlphaColorRec.Brown, 36);
  // DrawDebugVector(controller.Debug_pFuture, TAlphaColorRec.Brown, 40);
  DrawDebugVector(TWorldOracle.DebugPoint, TAlphaColorRec.Blue, 40);

  World.Step(0, 0, 0, True);

  EndPaint;
end;

procedure TMainForm.BeginPaint;
begin
  Scene.BeginScene;
  Scene.Clear(TAlphaColorRec.Black);
end;

procedure TMainForm.EndPaint;
begin
  Scene.EndScene;
end;

procedure TMainForm.ImageMouseUp(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Single);
begin
  MousePos:= TPointF.Create(X, Y);
  MouseStates[Button]:= False;
  if not MouseStates[TMouseButton.mbLeft] and (Button = TMouseButton.mbLeft) and Assigned(MouseJoint) then
  begin
    World.DestroyJoint(MouseJoint);
    MouseJoint := nil;
  end;
  UpdateMouse;
end;

procedure TMainForm.ImageMouseMove(Sender: TObject; Shift: TShiftState; X, Y: Single);
begin
  MousePos:= TPointF.Create(X, Y);
  UpdateMouse;
end;

type
  TQueryCallback = class(Tb2QueryCallback)
  public
     Point: TVector2;
     Fixture: Tb2Fixture;

     procedure Initizlize(const point: TVector2); {$IFDEF INLINE_AVAIL}inline;{$ENDIF}
     function ReportFixture(fixture: Tb2Fixture): Boolean; override;
  end;

{ TQueryCallback }

procedure TQueryCallback.Initizlize(const point: TVector2);
begin
  Self.Point:= point;
  Fixture:= nil;
end;

function TQueryCallback.ReportFixture(fixture: Tb2Fixture): Boolean;
begin
   if fixture.GetBody.GetType = b2_dynamicBody then
   begin
      if fixture.TestPoint(Point) then
      begin
         Self.Fixture := fixture;
         // We are done, terminate the query.
         Result := False;
         Exit;
      end;
   end;

   // Continue the query.
   Result := True;
end;

var
  QueryCallback: TQueryCallback;

procedure TMainForm.CreateMouseJoint;
var
  d, p: TVector2;
  aabb: Tb2AABB;
  body: Tb2Body;
  md: Tb2MouseJointDef;
begin
  p:= ToBox2D(MousePos);
  d.SetValue(0.001, 0.001);
  aabb.lowerBound:= p - d;
  aabb.upperBound:= p + d;

  QueryCallback.Initizlize(p);
  World.QueryAABB(QueryCallback, aabb);
  if Assigned(QueryCallback.Fixture) then
  begin
    body:= QueryCallback.Fixture.GetBody;
    md:= Tb2MouseJointDef.Create;
    md.bodyA:= MouseBody;
    md.bodyB:= body;
    md.target:= p;
    md.maxForce:= 1000.0 * body.GetMass;
    MouseJoint := Tb2MouseJoint(World.CreateJoint(md));
    body.SetAwake(True);
  end;
end;

procedure TMainForm.ImageMouseDown(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Single);
begin
  MousePos:= TPointF.Create(X, Y);
  MouseStates[Button]:= True;
  if MouseStates[TMouseButton.mbLeft] and (Button = TMouseButton.mbLeft) and (MouseJoint = nil) then
    CreateMouseJoint;
  UpdateMouse;
end;

procedure TMainForm.ImageMouseWheel(Sender: TObject; Shift: TShiftState; WheelDelta: Integer; var Handled: Boolean);
var
  OldScale: PhysicsFloat;
  OldSize, NewSize: TVector2;
begin
  OldScale:= Scale;
  if WheelDelta > 0 then
    Scale:= Scale * (WheelDelta / 110)
  else
  if WheelDelta < 0 then
    Scale:= Scale / (-WheelDelta / 110);
  OldSize:= TVector2.From(ClientWidth, -ClientHeight) / OldScale / 2;
  NewSize:= TVector2.From(ClientWidth, -ClientHeight) / Scale / 2;
  SceneZero:= SceneZero + (NewSize - OldSize);
end;

procedure TMainForm.UpdateMouse;
var
  D: TPointF;
  hip, knee: Integer;
begin
  if not LastMousePos.EqualsTo(EmptyPoint) then
    D:= MousePos - LastMousePos
  else D:= Default(TPointF);
  LastMousePos:= MousePos;

  if D.Length > 0 then
  begin
    D:= D / Scale;
    if MouseStates[TMouseButton.mbRight] then
      SceneZero:= SceneZero + TVector2.From(D.X, -D.Y);
  end;

	if Assigned(MouseJoint) then
    MouseJoint.SetTarget(ToBox2D(MousePos));

  if MouseStates[TMouseButton.mbMiddle] then
  begin
    hip:= character.getJointIndex('rHip');
    knee:= character.getJointIndex('rKnee');

		controller.DebugComputeIKSwingLegTargets(ToBox2D(MousePos));
    HipAngle:= controller.desiredPose.getJointRelativeOrientation(hip);
    KneeAngle:= controller.desiredPose.getJointRelativeOrientation(knee);
    character.Joints[hip].child.SetOrientation(HipAngle);
    character.Joints[knee].child.SetOrientation(HipAngle + KneeAngle);
    character.fixJointConstraints;
  end;
end;

{ TDestructionListener }

procedure TDestructionListener.SayGoodbye(fixture: Tb2Fixture);
begin
  raise ENotImplemented.Create('not done');
end;

procedure TDestructionListener.SayGoodbye(joint: Tb2Joint);
begin
  if MainForm.MouseJoint = joint then
    MainForm.MouseJoint:= nil
  else
    raise ENotImplemented.Create('not done');
end;

{ TContactFilter }

function TContactFilter.ShouldCollide(fixtureA, fixtureB: Tb2Fixture): Boolean;
var
  Obj1, Obj2: TObject;
  CartwheelBody1: TArticulatedRigidBody absolute Obj1;
  CartwheelBody2: TArticulatedRigidBody absolute Obj2;
begin
  Obj1:= TObject(fixtureA.GetBody.UserData);
  Obj2:= TObject(fixtureB.GetBody.UserData);

  // если объекты принадлежат cartwheel - он решает сталкиваются они или нет
  if (Obj1 is TArticulatedRigidBody) and (Obj2 is TArticulatedRigidBody) then
    Exit(TCartwheelWorld.shouldCheckForCollisions(CartwheelBody1, CartwheelBody2));

  // more checks could be added here
  Result:= True;
end;

{ TContactListener }

procedure TContactListener.BeginContact(var contact: Tb2Contact);
begin
end;

procedure TContactListener.EndContact(var contact: Tb2Contact);
begin
end;

procedure TContactListener.PreSolve(var contact: Tb2Contact; const oldManifold: Tb2Manifold);
begin
end;

procedure TContactListener.PostSolve(var contact: Tb2Contact; const impulse: Tb2ContactImpulse);
var
  Obj1, Obj2: TObject;
begin
  Obj1:= TObject(contact.m_fixtureA.GetBody.UserData);
  Obj2:= TObject(contact.m_fixtureB.GetBody.UserData);
  MainForm.CartwheelWorld.addContact(Obj1, Obj2, contact, impulse, OneStep);
end;

{ TDraw }

constructor TDraw.Create;
begin
  inherited;
  m_drawFlags:= [Tb2DrawBits.e_shapeBit, Tb2DrawBits.e_jointBit];
end;

function Vec2Point(const v: TVector2) : TPointF;
begin
  Result:= TPointF.Create(v.x, -v.y);
end;

function ToBGRA(const c: RGBA) : TAlphaColor; overload;
var
  Rec: TAlphaColorRec absolute Result;
begin
  Rec.R:= Trunc(c[0] * $FF);
  Rec.G:= Trunc(c[1] * $FF);
  Rec.B:= Trunc(c[2] * $FF);
  Rec.A:= Trunc(c[3] * $FF);
end;

function ToBGRA(const c: RGBA; alpha_mul: Single) : TAlphaColor; overload;
var
  Rec: TAlphaColorRec absolute Result;
begin
  Rec.R:= Trunc(c[0] * $FF);
  Rec.G:= Trunc(c[1] * $FF);
  Rec.B:= Trunc(c[2] * $FF);
  Rec.A:= Trunc(c[3] * alpha_mul * $FF);
end;

procedure TDraw.DrawSegment(const p1, p2: TVector2; const color: RGBA);
var
  pp1, pp2: TPointF;
begin
  pp1:= TMainForm.ToScene(p1);
  pp2:= TMainForm.ToScene(p2);
  MainForm.Scene.StrokeThickness:= 2.0;
  MainForm.Scene.Stroke.Color:= ToBGRA(color);
  MainForm.Scene.DrawLine(pp1, pp2, 1);
end;

procedure TDraw.DrawSolidCircle(const center, axis: TVector2; radius: PhysicsFloat; const color: RGBA);
var
  CenterPoint, AxisPoint, AxisVector: TPointF;
  EllipseRect: TRectF;
begin
  CenterPoint:= TMainForm.ToScene(center);
  radius:= radius * MainForm.Scale;
  EllipseRect:= TRectF.Create(CenterPoint.X - radius, CenterPoint.Y - radius, CenterPoint.X + radius, CenterPoint.Y + radius);

  MainForm.Scene.Fill.Color:= ToBGRA(color, 0.25);
  MainForm.Scene.FillEllipse(EllipseRect, 1);
  MainForm.Scene.StrokeThickness:= 2.0;
  MainForm.Scene.Stroke.Color:= ToBGRA(color);
  MainForm.Scene.DrawEllipse(EllipseRect, 1);

  AxisVector:= TPointF.Create(axis.X, -axis.Y);
  AxisPoint := CenterPoint + AxisVector * radius;
  MainForm.Scene.DrawLine(CenterPoint, AxisPoint, 1);
end;

procedure TDraw.DrawSolidPolygon(const vertices: Tb2PolyVertices; vertexCount: Int32; const color: RGBA);
var
  Polygon: TPolygon;
  index: Integer;
begin
  SetLength(Polygon, vertexCount + 1);
  for index := 0 to vertexCount - 1 do
    Polygon[index]:= TMainForm.ToScene(vertices[index]);
  Polygon[vertexCount]:= Polygon[0];
  MainForm.Scene.Fill.Color:= ToBGRA(color, 0.25);
  MainForm.Scene.FillPolygon(Polygon, 1);
  MainForm.Scene.StrokeThickness:= 2.0;
  MainForm.Scene.Stroke.Color:= ToBGRA(color);
  MainForm.Scene.DrawPolygon(Polygon, 1);
end;

procedure TDraw.DrawTransform(const xf: Tb2Transform);
const
  k_axisScale = 0.1;
var
  p1, p2, xAxis, yAxis: TVector2;
begin
  xAxis := xf.q.GetXAxis;
  yAxis := xf.q.GetYAxis;
  p1:= xf.p;
  p2.x := p1.x + k_axisScale * xAxis.x;
  p2.y := p1.y + k_axisScale * xAxis.y;

  MainForm.Scene.Stroke.Color:= TAlphaColorRec.Red;
  MainForm.Scene.DrawLine(TMainForm.ToScene(p1), TMainForm.ToScene(p2), 1);

  p2.x := p1.x + k_axisScale * yAxis.x;
  p2.y := p1.y + k_axisScale * yAxis.y;

  MainForm.Scene.Stroke.Color:= TAlphaColorRec.Green;
  MainForm.Scene.DrawLine(TMainForm.ToScene(p1), TMainForm.ToScene(p2), 1);
end;

// coordinate convert utils

class function TMainForm.ToScene(V: TVector2) : TPointF;
begin
  V:= V + MainForm.SceneZero;
  Result:= TPointF.Create(v.x, -v.y) * MainForm.Scale;
end;

class function TMainForm.ToBox2D(P: TPointF) : TVector2;
begin
  P:= P / MainForm.Scale;
  Result:= TVector2.From(P.X, -P.Y) - MainForm.SceneZero;
end;

{ timing }

var
  vCounterFrequency: Int64;

function GetRawReferenceTime: Double;
var
   counter: Int64;
begin
   QueryPerformanceCounter(counter);
   Result := counter / vCounterFrequency;
end;

initialization
  if not QueryPerformanceFrequency(vCounterFrequency) then
    vCounterFrequency := 0;
  QueryCallback := TQueryCallback.Create;
finalization
  QueryCallback.Free;
end.

unit Cartwheel.WorldOracle;

interface

uses
  Box2D.Physics2D, Box2D.Physics2DTypes,
  Cartwheel.Character;

type
  TWorldOracle = class
  strict private
    World: Tb2World;
  public
    class var
      DebugPoint, DebugPoint1, DebugPoint2: TVector2;
    procedure initializeWorld(physicalWorld: Tb2World);
    function getWorldHeightAt(SelfFigure: TArticulatedFigure; const worldLoc: TVector2) : Double;
  end;

implementation

uses
  Cartwheel.Controllers;

type
  TWorldOracleRayCastCallback = class(Tb2RayCastCallback)
  public
    SelfFigure: TArticulatedFigure;
    hit: Boolean;
    point: TVector2;
    constructor Create(SelfFigure: TArticulatedFigure);
    function ReportFixture(fixture:	Tb2Fixture; const point, normal: TVector2; fraction: PhysicsFloat): PhysicsFloat; override;
  end;

{ TWorldOracleRayCastCallback }

constructor TWorldOracleRayCastCallback.Create(SelfFigure: TArticulatedFigure);
begin
  Self.SelfFigure:= SelfFigure;
  inherited Create;
end;

function TWorldOracleRayCastCallback.ReportFixture(fixture: Tb2Fixture; const point, normal: TVector2;
  fraction: PhysicsFloat): PhysicsFloat;
var
  UserData: TObject;
  Body: TArticulatedRigidBody absolute UserData;
begin
  UserData := TObject(fixture.GetBody.UserData);
  if (UserData is TArticulatedRigidBody) and (Body.AFParent = SelfFigure) then
     Result := -1.0 // filter
  else
  begin
    Self.hit:= True;
    Self.point:= point;

    Result:= fraction;
  end;
end;

{ TWorldOracle }

procedure TWorldOracle.initializeWorld(physicalWorld: Tb2World);
begin
  World:= physicalWorld;
end;

function TWorldOracle.getWorldHeightAt(SelfFigure: TArticulatedFigure; const worldLoc: TVector2): Double;
var
  callback: TWorldOracleRayCastCallback;
  point1, point2: TVector2;
begin
  // Exit(0);

  callback:= TWorldOracleRayCastCallback.Create(SelfFigure);
  point1:= worldLoc + Up * 0.3;
  point2:= worldLoc - Up * 0.3;
  World.RayCast(callback, point1, point2);
  if callback.hit then
    Result:= callback.point.y
  else Result:= point2.y;
  callback.Free;

  DebugPoint:= TVector2.From(point1.x, Result);
  DebugPoint1:= point1;
  DebugPoint2:= point2;
end;

end.

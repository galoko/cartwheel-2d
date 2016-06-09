unit Cartwheel.WorldOracle;

interface

uses
  Box2D.Physics2D, Box2D.Physics2DTypes;

type
  TWorldOracle = class
  strict private
    World: Tb2World;
  public
    class var
      DebugPoint: TVector2;
    procedure initializeWorld(physicalWorld: Tb2World);
    function getWorldHeightAt(const worldLoc: TVector2) : Double;
  end;

implementation

uses
  Cartwheel.Controllers, Cartwheel.Character;

type
  TWorldOracleRayCastCallback = class(Tb2RayCastCallback)
  public
    hit: Boolean;
    point: TVector2;
    function ReportFixture(fixture:	Tb2Fixture; const point, normal: TVector2; fraction: PhysicsFloat): PhysicsFloat; override;
  end;

{ TWorldOracleRayCastCallback }

function TWorldOracleRayCastCallback.ReportFixture(fixture: Tb2Fixture; const point, normal: TVector2;
  fraction: PhysicsFloat): PhysicsFloat;
var
  userData: TObject;
begin
  userData := TObject(fixture.GetBody.UserData);
  if userData is TArticulatedRigidBody then
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

function TWorldOracle.getWorldHeightAt(const worldLoc: TVector2): Double;
var
  callback: TWorldOracleRayCastCallback;
  point1, point2: TVector2;
begin
  // Exit(0);

  callback:= TWorldOracleRayCastCallback.Create;
  point1:= worldLoc + Up * 2;
  point2:= worldLoc - Up * 2;
  World.RayCast(callback, point1, point2);
  if callback.hit then
    Result:= callback.point.y
  else Result:= point2.y;
  callback.Free;

  DebugPoint:= TVector2.From(point1.x, Result);
end;

end.

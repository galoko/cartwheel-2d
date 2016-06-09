unit Utils.Trajectory;

interface

uses
  System.SysUtils, System.Math, System.Generics.Collections, System.Generics.Defaults;

type
  TTrajectory1d = class
  private
    const
      TINY_NUMBER = 0.000000001;
      TINY = TINY_NUMBER;
    type
      TTrajectory1dItem = record
        t, value: Double;
      end;
    var
      lastIndex: Integer;
      Items: TList<TTrajectory1dItem>;
    function getFirstLargerIndex(t: Double) : Integer;
  public
    constructor Create;
    destructor Destroy; override;
    function getKnotValue(i: Integer) : Double;
    function getKnotPosition(i: Integer) : Double;
    procedure setKnotValue(i: Integer; val: Double);
    procedure setKnotPosition(i: Integer; pos: Double);
    function getMinPosition : Double;
    function getMaxPosition : Double;
    function getKnotCount : Integer;
    procedure addKnot(t, val: Double);
    procedure removeKnot(i: Integer);
    procedure clear;
    procedure copy(other: TTrajectory1d);

    function evaluate_linear(t: Double) : Double;
    function evaluate_catmull_rom(t: Double) : Double;
    procedure simplify_catmull_rom(maxError: Double; nbSamples: Integer = 100);
  class var
    Fancy: Boolean;
  end;


implementation

{ TTrajectory1d }

constructor TTrajectory1d.Create;
begin
  Items:= TList<TTrajectory1dItem>.Create;
end;

destructor TTrajectory1d.Destroy;
begin
  FreeAndNil(Items);
  inherited;
end;

function TTrajectory1d.getFirstLargerIndex(t: Double): Integer;
var
  size, i, index: Integer;
begin
  size:= Items.Count;
  if size = 0 then
    Exit(0);
  if t < Items[(lastIndex+size-1) mod size].t then
    lastIndex:= 0;
  for i:= 0 to size - 1 do
  begin
    index:= (i + lastIndex) mod size;
    if t < Items[index].t then
    begin
      lastIndex:= index;
      Exit(index);
    end;
  end;
  Result:= size;
end;

function TTrajectory1d.getKnotValue(i: Integer): Double;
begin
  Result:= Items[i].value;
end;

function TTrajectory1d.getKnotPosition(i: Integer): Double;
begin
  Result:= Items[i].t;
end;

procedure TTrajectory1d.setKnotValue(i: Integer; val: Double);
var
  Item: TTrajectory1dItem;
begin
  Item.t:= Items[i].t;
  Item.value:= val;
  Items[i]:= Item;
end;

procedure TTrajectory1d.setKnotPosition(i: Integer; pos: Double);
var
  Item: TTrajectory1dItem;
begin
  if (i-1 >= 0) and (Items[i-1].t >= pos) then Exit;
  if ((i+1) < Items.Count - 1) and (Items[i+1].t <= pos) then Exit;
  Item.t:= pos;
  Item.value:= Items[i].value;
  Items[i]:= Item;
end;

function TTrajectory1d.getMinPosition: Double;
begin
  if Items.Count = 0 then
    Exit(Infinity);
  Result:= Items.First.t;
end;

function TTrajectory1d.getMaxPosition: Double;
begin
  if Items.Count = 0 then
    Exit(NegInfinity);
  Result:= Items.Last.t;
end;

function TTrajectory1d.getKnotCount: Integer;
begin
  Result:= Items.Count;
end;

procedure TTrajectory1d.addKnot(t, val: Double);
var
  index: Integer;
  Item: TTrajectory1dItem;
begin
  index:= getFirstLargerIndex(t);

  Item.t:= t;
  Item.value:= val;
  Items.Insert(index, Item);
end;

procedure TTrajectory1d.removeKnot(i: Integer);
begin
  Items.Delete(i);
end;

procedure TTrajectory1d.clear;
begin
  Items.Clear;
end;

procedure TTrajectory1d.copy(other: TTrajectory1d);
begin
  Items.Clear;
  Items.AddRange(other.Items);
end;

function TTrajectory1d.evaluate_linear(t: Double): Double;
var
  size, index: Integer;
begin
  size:= Items.Count;
  if size = 0 then Exit(0);
  if t <= Items[0].t then Exit(Items[0].value);
  if t >= Items[size-1].t then	Exit(Items[size-1].value);
  index:= getFirstLargerIndex(t);

  //now linearly interpolate between inedx-1 and index
  t:= (t-Items[index-1].t) / (Items[index].t-Items[index-1].t);
  Result:= Items[index-1].value * (1-t) + Items[index].value * t;
end;

function TTrajectory1d.evaluate_catmull_rom(t: Double): Double;
var
  size, index: Integer;
  t0, t1, t2, t3,
  p0, p1, p2, p3,
  d1, d2,
  m1, m2: Double;
begin
  size:= Items.Count;
  if size = 0 then Exit(0);
  if t <= Items[0].t then Exit(Items[0].value);
  if t >= Items[size-1].t	then Exit(Items[size-1].value);
  index:= getFirstLargerIndex(t);

  //now that we found the interval, get a value that indicates how far we are along it
  t:= (t-Items[index-1].t) / (Items[index].t-Items[index-1].t);

  //approximate the derivatives at the two ends
  if index-2 < 0 then p0:= Items[index-1].value else p0:= Items[index-2].value;
  p1:= Items[index-1].value;
  p2:= Items[index].value;
  if index+1>=size then p3:= Items[index].value else p3:= Items[index+1].value;

  if index-2<0 then t0:= Items[index-1].t else t0:= Items[index-2].t;
  t1:= Items[index-1].t;
  t2:= Items[index].t;
  if index+1>=size then t3:= Items[index].t else t3:= Items[index+1].t;

  d1:= (t2-t0);
  d2:= (t3-t1);

  if (d1 > -TINY) and (d1  <  0) then d1:= -TINY;
  if (d1 <  TINY) and (d1  >= 0) then d1:=  TINY;
  if (d2 > -TINY) and (d2  <  0) then d2:= -TINY;
  if (d2 <  TINY) and (d2  >= 0) then d2:=  TINY;

  if Fancy then
  begin
    m1:= (p2 - p0) * (1-(t1-t0)/d1);
    m2:= (p3 - p1) * (1-(t3-t2)/d2);
  end
  else
  begin
    m1:= (p2 - p0)*0.5;
    m2:= (p3 - p1)*0.5;
  end;

  t2:= t*t;
  t3:= t2*t;

  //and now perform the interpolation using the four hermite basis functions from wikipedia
  Result:= p1*(2*t3-3*t2+1) + m1*(t3-2*t2+t) + p2*(-2*t3+3*t2) + m2 * (t3 - t2);
end;

procedure TTrajectory1d.simplify_catmull_rom(maxError: Double; nbSamples: Integer);
var
  startTime, endTime: Double;
  result: TTrajectory1d;
  currError, currErrorTime: Double;
  i: Integer;
  interp, time, error: Double;
begin
  if getKnotCount < 3 then Exit;

  startTime:= Items.First.t;
  endTime:= Items.Last.t;

  result:= TTrajectory1d.Create;
  try
    result.addKnot(startTime, Items.First.value);
    result.addKnot(endTime, Items.Last.value);


    while True do
    begin
      currError:= 0;
      currErrorTime:= NegInfinity;

      for i:= 0 to nbSamples - 1 do
      begin
        interp:= i / (nbSamples - 1.0);
        time:= startTime * (1 - interp) + endTime * interp;
        error:= Abs( result.evaluate_catmull_rom(time) - evaluate_catmull_rom(time) );
        if error > currError then
        begin
          currError:= error;
          currErrorTime:= time;
        end;
      end;

      if currError <= maxError then
        break;

      result.addKnot(currErrorTime, evaluate_catmull_rom(currErrorTime));
    end;

    copy(result);
  finally
    FreeAndNil(Result);
  end;
end;

end.

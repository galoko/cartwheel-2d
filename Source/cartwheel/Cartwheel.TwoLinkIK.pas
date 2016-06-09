unit Cartwheel.TwoLinkIK;

interface

uses
  System.Math,
  Box2D.Physics2DTypes,
  Cartwheel.Character;

type
  TwoLinkIK = class
  public
    class var
      DebugPoint: TVector2;
    class function solve(const p1, p2: TVector2; n, r1, r2: Double) : TVector2; static;
    class procedure getIKOrientations(const p1, p2: TVector2; n: Double; const vParent: TVector2; nParent: Double; const vChild: TVector2;
      var aP, aC: Double); static;
    class function getParentOrientation(const vGlobal: TVector2; nGlobal: Double; const vLocal: TVector2; nLocal: Double) : Double; static;
    class function getChildRotationAngle(const vParent, vChild: TVector2; n: Double) : Double; static;
  end;

implementation

{ TwoLinkIK }

class function TwoLinkIK.solve(const p1, p2: TVector2; n, r1, r2: Double): TVector2;
var
  r, a, tmp, h: Double;
  d1, d2, p: TVector2;
begin
  r:= VectorDelta(p1, p2).length();
  if r > (r1 + r2) * 0.993 then
    r:= (r1 + r2) * 0.993;
  //this is the length of the vector starting at p1 and going to the midpoint between p1 and p2
  a:= (r1 * r1 - r2 * r2 + r * r) / (2 * r);
  tmp:= r1*r1 - a*a;
  if tmp < 0 then
    tmp:= 0;
  //and this is the distance from the midpoint of p1-p2 to the intersection point
  h:= sqrt(tmp);
  //now we need to get the two directions needed to reconstruct the intersection point
  d1:= VectorDelta(p1, p2);
  d1.Normalize;
  d2:= b2Cross(d1, n);
  d2.Normalize;

  //and now get the intersection point
  p:= p1 + d1 * a + d2 * (-h);

  Result:= p;
end;

function safeACOS(val: Double) : Double; inline;
begin
	if val<-1 then
		Exit(PI);
	if val>1 then
		Exit(0);
	Result:= ArcCos(val);
end;

function angleWith(const v1, v2: TVector2) : Double; inline;
begin
  Result:= b2Dot(v1, v2) / (v1.length() * v2.length());
  result:= safeACOS(result);
end;

class function TwoLinkIK.getParentOrientation(const vGlobal: TVector2; nGlobal: Double; const vLocal: TVector2; nLocal: Double): Double;
var
  axis, a, ang: Double;
begin
  axis:= Sign(b2Cross(vLocal, vGlobal));
  ang:= angleWith(vLocal, vGlobal);

  a:= ang * axis;

  Result:= a;
end;

class function TwoLinkIK.getChildRotationAngle(const vParent, vChild: TVector2; n: Double): Double;
var
  angle: Double;
begin
  //compute the angle between the vectors (p1, p) and (p, p2), and that's our result
  angle:= angleWith(vParent, vChild);
  if b2Cross(vParent, vChild) * n < 0 then
    angle:= -angle;

  Result:= angle;
end;

class procedure TwoLinkIK.getIKOrientations(const p1, p2: TVector2; n: Double; const vParent: TVector2; nParent: Double;
  const vChild: TVector2; var aP, aC: Double);
var
  nG: Double;
  solvedJointPosW, vParentG, vChildG: TVector2;
  childAngle: Double;
begin
	nG:= n;

	solvedJointPosW:= solve(p1, p2, nG, vParent.length, vChild.length);

  DebugPoint:= solvedJointPosW;

	vParentG:= VectorDelta(p1, solvedJointPosW);
	vChildG:= VectorDelta(solvedJointPosW, p2);

	aP:= getParentOrientation(vParentG, nG, vParent, nParent);

	childAngle:= getChildRotationAngle(vParentG, vChildG, nG);
	aC:= childAngle * nParent;
end;

end.

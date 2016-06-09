program cartwheel2d;

uses
  Winapi.Windows,
  FMX.Forms,
  FMX.Types,
  Vcl.MainForm in 'Vcl.MainForm.pas' {MainForm},
  Box2D.Physics2D in 'Physics2D\Box2D.Physics2D.pas',
  Box2D.Physics2DControllers in 'Physics2D\Box2D.Physics2DControllers.pas',
  Box2D.Physics2DHelper in 'Physics2D\Box2D.Physics2DHelper.pas',
  Box2D.Physics2DPolygonTool in 'Physics2D\Box2D.Physics2DPolygonTool.pas',
  Box2D.Physics2DTypes in 'Physics2D\Box2D.Physics2DTypes.pas',
  MSTimer in 'Physics2D\MSTimer.pas',
  Cartwheel.CharacterDescription in 'cartwheel\Cartwheel.CharacterDescription.pas',
  Cartwheel.Character in 'cartwheel\Cartwheel.Character.pas',
  Cartwheel.World in 'cartwheel\Cartwheel.World.pas',
  Cartwheel.Controllers in 'cartwheel\Cartwheel.Controllers.pas',
  Utils.Trajectory in 'cartwheel\Utils.Trajectory.pas',
  Cartwheel.EditableWalk in 'cartwheel\Cartwheel.EditableWalk.pas',
  Cartwheel.TwoLinkIK in 'cartwheel\Cartwheel.TwoLinkIK.pas',
  Cartwheel.WorldOracle in 'cartwheel\Cartwheel.WorldOracle.pas';

{$R *.res}

begin
  // GlobalUseGPUCanvas:= True;
  // AllocConsole;
  Application.Initialize;
  Application.CreateForm(TMainForm, MainForm);
  Application.Run;
end.

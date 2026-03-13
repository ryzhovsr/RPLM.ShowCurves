#include "RPLM.CAD.UI.СonjugationCurves.Command.h"
#include "RPLM.CAD.UI.ConjugationCurves.Resources.h"
#include "RPLM.CAD.CurveParser.h"
#include "Model/Objects/RGPBodyObject.h"
#include "Generators/BodyConstructor.h"
#include "Model/Representations/RGPModelScene.h"
#include "RPLM.EP.Model/Model/Representations/RGPPresentationContexts.h"
#include "RGPSession.h"

namespace RPLM::CAD
{
	namespace UI
	{
		RPLMCADShowCurvesCommand::RPLMCADShowCurvesCommand() :
			_sourceCurvesFilePath(L"SourceCurvesFilePath", RSCADUIW("RPLM.CAD.FileWithSourceCurves"), L""),
			_showSourceCurves(L"ShowSourceCurves", RSCADUIW("RPLM.CAD.ShowSourceCurves"), L"", false, true),
			_curveDegree(L"CurveDegree", RSCADUIW("CurveDegree")),
			_controlPointsFilePath(L"ControlPoints", RSCADUIW("ControlPoints"), L""),
			_knotsFilePath(L"Knots", RSCADUIW("Knots"), L""),
			_fixBeginningCurve(L"FixBeginningCurve", RSCADUIW("FixBeginningCurve"), L"", false, true),
			_fixEndCurve(L"FixEndCurve", RSCADUIW("FixEndCurve"), L"", false, true),
			_saveConjugatedCurveInFile(L"SaveConjugatedCurveInFile", RSCADUIW("RPLM.CAD.SaveConjugatedCurveInFile"), L"", false, true),
			_conjugatedCurveFilePath(L"ConjugatedCurveFilePath", RSCADUIW("RPLM.CAD.ConjugatedCurveFilePath"), L"")
		{
			_dialog.SetTitle(RSCADUIW("RPLM.CAD.UI.ConjugationCurves"));

			AddOkToDialog(&_dialog);
			AddCancelToDialog(&_dialog);

			_dialog.AddControl(_sourceCurvesFilePath);
			_dialog.AddControl(_showSourceCurves);

			// Чекбоксы фиксации начала и конца кривой
			_dialog.AddControl(_fixBeginningCurve);
			_dialog.AddControl(_fixEndCurve);

			// Чекбокс сохранения сопряжённой кривой в файл
			_dialog.AddControl(_saveConjugatedCurveInFile);
			// Путь к файлу для сохранения сопряжённой кривой
			_dialog.AddControl(_conjugatedCurveFilePath);
			_conjugatedCurveFilePath.SetHidden(true);

			_ok.PressEvent = std::bind(&RPLMCADShowCurvesCommand::OnOK, this);
			_dialog.OnCloseEvent = std::bind(&RPLMCADShowCurvesCommand::OnCloseDialog, this);
			_sourceCurvesFilePath.LinkChanged = std::bind(&RPLMCADShowCurvesCommand::OnFilePathChanged, this);
			_conjugatedCurveFilePath.LinkChanged = std::bind(&RPLMCADShowCurvesCommand::OnFilePathChanged, this);
		}

		RPLMCADShowCurvesCommand::~RPLMCADShowCurvesCommand()
		{
		}

		bool RPLMCADShowCurvesCommand::Start(EP::UI::StartCommandParameters& iParameters)
		{
			if (!Command::Start(iParameters))
				return false;

			CreateCommandDialog(_dialog, GetMainWindow(), GetDocument());
			_dialog.NeedToAdjust();
			_dialog.Show();

			_ok.SetEnabled(IsOkEnabled());

			return true;
		}

		void RPLMCADShowCurvesCommand::Finish()
		{
			_dialog.Destroy();
			Command::Finish();
		}

		RPLM::EP::UI::ControlLayout* RPLMCADShowCurvesCommand::GetDialog()
		{
			return &_dialog;
		}

		std::string RPLMCADShowCurvesCommand::GetID()
		{
			return "RPLM.CAD.ConjugationCurves";
		}

		void RPLMCADShowCurvesCommand::OnOK()
		{
			Base::Framework::String sourceCurvesFilePath = _sourceCurvesFilePath.GetFullName();

			if (sourceCurvesFilePath.empty())
			{
				EP::UI::Command::Alert(L"Пустой путь к файлу.", AlertType::Error);
				return;
			}

			std::vector<RGK::NURBSCurve> curves = CAD::CurveParser::ReadCurvesFromFile(sourceCurvesFilePath);

			if (curves.empty())
			{
				EP::UI::Command::Alert(L"Ошибка чтения кривых из файла.", AlertType::Error);
				return;
			}

			if (_showSourceCurves.IsChecked())
			{
				for (const auto& curve : curves)
				{
					if (DrawCurve(curve) != RGK::Success)
					{
						EP::UI::Command::Alert(L"Ошибка отображения кривой на сцене.", AlertType::Error);
					}
				}
			}

			for (const auto& curve : curves)
			{
				if (DrawCurve(curve) != RGK::Success)
				{
					EP::UI::Command::Alert(L"Ошибка отображения кривой на сцене.", AlertType::Error);
				}
			}

			Terminate();
		}

		bool RPLMCADShowCurvesCommand::OnCloseDialog()
		{
			Terminate();
			return false;
		}

		void RPLMCADShowCurvesCommand::OnFilePathChanged()
		{
			_ok.SetEnabled(IsOkEnabled());
		}

		bool RPLMCADShowCurvesCommand::IsOkEnabled()
		{
			bool isSourceCurvesFilePathValid = IsFilePathValid(_sourceCurvesFilePath.GetFullName());
			bool isConjugatedCurveFilePathValid = true;

			// Если активирован чекбокс сохранения сопряжённой кривой в файл
			if (_saveConjugatedCurveInFile.IsChecked())
			{
				isConjugatedCurveFilePathValid = IsFilePathValid(_conjugatedCurveFilePath.GetFullName());
			}

			return isSourceCurvesFilePathValid && isConjugatedCurveFilePathValid;
		}

		bool RPLMCADShowCurvesCommand::IsFilePathValid(const Base::Framework::String& iFilePath)
		{
			// 1. Проверка на пустой путь
			if (iFilePath.empty())
			{
				return false;
			}

			// 2. Проверка существования файла и возможности открытия
			std::ifstream inStream(iFilePath.c_str());

			if (!inStream.is_open())
			{
				return false;
			}

			// 3. Проверка расширения файла 
			std::wstring filePath = iFilePath;
			size_t dotPos = filePath.rfind(L'.');

			if (dotPos != std::wstring::npos)
			{
				std::wstring extension = filePath.substr(dotPos);
				std::wstring allowedExtensions[] = { L".txt" };
				bool validExtension = false;

				for (const auto& ext : allowedExtensions)
				{
					if (_wcsicmp(extension.c_str(), ext.c_str()) == 0)
					{
						validExtension = true;
						break;
					}
				}

				if (!validExtension)
				{
					inStream.close();
					return false;
				}
			}
			else
			{
				// Файл без расширения
				inStream.close();
				return false;
			}

			inStream.close();

			return true;
		}

		RGK::Result RPLMCADShowCurvesCommand::DrawCurve(const RGK::NURBSCurve& iCurve) const
		{
			if (!iCurve)
				return RGK::Result::NullPointer;

			RGK::Context rgkContext;
			RPLM::EP::Model::Session::GetSession()->GetRGKSession().CreateMainContext(rgkContext);

			RGK::BodyConstructor::Data data(0, RGK::Body::Type::Wire);
			data.CreateCoEdgeParametricCurvesAutomatically(true);

			RGK::Math::Vector3DArray controlPoints = iCurve.GetControlPoints();
			std::vector<double> weights = iCurve.GetWeights();
			std::vector<double> knots = iCurve.GetKnots();
			int degree = iCurve.GetDegree();
			bool isPeriodic = iCurve.IsPeriodic();

			const auto tolerance = rgkContext.GetLinearPrecision();
			data.AddVertex(controlPoints.front(), tolerance);
			data.AddVertex(controlPoints.back(), tolerance);

			int ends[2] = { 0, 1 };
			double interval[2] = { knots.front(), knots.back() };
			data.AddEdge(ends, iCurve, true, interval, tolerance);

			RGK::BodyConstructor::Report report;
			auto resultCreationBody = RGK::BodyConstructor::Create(rgkContext, data, report);

			if (resultCreationBody != RGK::Success)
				return resultCreationBody;

			RPLM::EP::Model::ModelScenePtr modelScene = nullptr;

			for (auto i = 0; i < GetDocument()->Representations().Size(); ++i)
			{
				auto presentation = GetDocument()->Representations()[i];

				if (!presentation)
					continue;

				if (presentation->IsTypeOf(RPLM::EP::Model::ModelScene::ClassID()))
				{
					modelScene = EP::Model::Cast<RPLM::EP::Model::ModelScene>(presentation);
					break;
				}
			}

			if (!modelScene)
				return RGK::Result::NullPointer;

			RPLM::EP::Model::EditDocument edit(GetDocument(), RSCADUIW("RPLM.CAD.DrawCurve"));
			{
				auto bodyObject = std::make_shared<RPLM::EP::Model::BodyObject>((report.GetBody()));
				GetDocument()->Objects().AddObject(bodyObject);
				modelScene->EditReferences()->AddObject(bodyObject);
				RPLM::EP::Model::Regeneration::RegenerationContext regenerationContext(GetDocument(), &rgkContext);
				modelScene->Update(RPLM::EP::Model::PresentationUpdateContext(&regenerationContext));
			}

			edit.End(false);
			return RGK::Success;
		}
	}
}
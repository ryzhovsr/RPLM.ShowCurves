#include "RPLM.CAD.UI.ShowCurves.Command.h"
#include "RPLM.CAD.UI.ShowCurves.Resources.h"
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
			_numberOfCurvesFromFile(L"NumberOfCurvesFromFile", RSCADUIW("RPLM.CAD.NumberOfCurvesFromFile"))
		{
			_dialog.SetTitle(RSCADUIW("RPLM.CAD.UI.ShowCurves"));

			AddOkToDialog(&_dialog);
			AddCancelToDialog(&_dialog);

			_dialog.AddControl(_sourceCurvesFilePath);
			_dialog.AddControl(_numberOfCurvesFromFile);
			_numberOfCurvesFromFile.SetReadOnly(true);
			_numberOfCurvesFromFile.SetPlaceholderText(L"0");

			_ok.PressEvent = std::bind(&RPLMCADShowCurvesCommand::OnOK, this);
			_dialog.OnCloseEvent = std::bind(&RPLMCADShowCurvesCommand::OnCloseDialog, this);
			_sourceCurvesFilePath.LinkChanged = std::bind(&RPLMCADShowCurvesCommand::OnFilePathChanged, this);
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
			return "RPLM.CAD.ShowCurves";
		}

		void RPLMCADShowCurvesCommand::OnOK()
        {
            for (const auto& curve : _sourceCurves)
            {
                if (RGK::Failed(DrawCurve(curve)))
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
			
			if (!IsOkEnabled())
			{
				_numberOfCurvesFromFile.SetPlaceholderText(L"0");
				return;
			}

			Base::Framework::String sourceCurvesFilePath = _sourceCurvesFilePath.GetFullName();

			if (sourceCurvesFilePath.empty())
			{
				EP::UI::Command::Alert(L"Пустой путь к файлу.", AlertType::Error);
				return;
			}

			auto result = CAD::CurveParser::ReadCurvesFromFile(sourceCurvesFilePath, _sourceCurves);

			if (result.codeError != CAD::CurveParser::ParserResult::SuccessReadFile)
			{
				std::string errorMessage;

				switch (result.codeError)
				{

				case CAD::CurveParser::ParserResult::ErrorOpenFile:
					errorMessage = "Ошибка открытия файла.";
					break;
				case CAD::CurveParser::ParserResult::UnexpectedBlock:
					errorMessage = "Встречен неожиданный/неизвестный блок (Строка " + std::to_string(result.lineNumber) + ").";
					break;
				case CAD::CurveParser::ParserResult::IncorrectFormatBlockDegree:
					errorMessage = "Некорректный формат блока Degree (Строка " + std::to_string(result.lineNumber) + ").";
					break;
				case CAD::CurveParser::ParserResult::DegreeParameterCannotBeNegative:
					errorMessage = "Параметр Degree не может быть отрицательным (Строка " + std::to_string(result.lineNumber) + ").";
					break;
				case CAD::CurveParser::ParserResult::IsPeriodicParameterIncorrect:
					errorMessage = "Некорректное значение IsPeriodic, допустимо только true/false/1/0 (Строка " + std::to_string(result.lineNumber) + ").";
					break;
				case CAD::CurveParser::ParserResult::IncorrectFormatBlockControlPoints:
					errorMessage = "Некорректный формат блока Control Points (Строка " + std::to_string(result.lineNumber) + ").";
					break;
				case CAD::CurveParser::ParserResult::IncorrectNumberExpectControlPoints:
					errorMessage = "Некорректное количество ожидаемых контрольных точек (Строка " + std::to_string(result.lineNumber) + ").";
					break;
				case CAD::CurveParser::ParserResult::ErrorCoordinateControlPoint:
					errorMessage = "Ошибка чтения координат контрольной точки (Строка " + std::to_string(result.lineNumber) + ").";
					break;
				case CAD::CurveParser::ParserResult::NumberActualControlPointsNotMatchExpect:
					errorMessage = "Количество контрольных точек не совпадает с ожидаемым (Строка " + std::to_string(result.lineNumber) + ").";
					break;
				case CAD::CurveParser::ParserResult::IncorrectFormatBlockWeights:
					errorMessage = "Некорректный формат блока Weights (Строка " + std::to_string(result.lineNumber) + ").";
					break;
				case CAD::CurveParser::ParserResult::IncorrectNumberExpectWeights:
					errorMessage = "Некорректное количество ожидаемых весовых коэффициентов (Строка " + std::to_string(result.lineNumber) + ").";
					break;
				case CAD::CurveParser::ParserResult::ErrorWeight:
					errorMessage = "Ошибка чтения весового коэффициента (Строка " + std::to_string(result.lineNumber) + ").";
					break;
				case CAD::CurveParser::ParserResult::NumberActualWeightsNotMatchExpect:
					errorMessage = "Количество весовых коэффициентов не совпадает с ожидаемым (Строка " + std::to_string(result.lineNumber) + ").";
					break;
				case CAD::CurveParser::ParserResult::IncorrectFormatBlockKnots:
					errorMessage = "Некорректный формат блока Knots (Строка " + std::to_string(result.lineNumber) + ").";
					break;
				case CAD::CurveParser::ParserResult::IncorrectNumberExpectKnots:
					errorMessage = "Некорректное количество ожидаемых узловых коэффициентов (Строка " + std::to_string(result.lineNumber) + ").";
					break;
				case CAD::CurveParser::ParserResult::ErrorKnot:
					errorMessage = "Ошибка чтения узлового коэффициента (Строка " + std::to_string(result.lineNumber) + ").";
					break;
				case CAD::CurveParser::ParserResult::NumberActualKnotsNotMatchExpect:
					errorMessage = "Количество узловых коэффициентов не совпадает с ожидаемым (Строка " + std::to_string(result.lineNumber) + ").";
					break;
				}

				EP::UI::Command::Alert(Base::Framework::ConvertStringToWstring(errorMessage), AlertType::Error);
			}

			_numberOfCurvesFromFile.SetPlaceholderText(Base::Framework::ConvertStringToWstring(std::to_string(_sourceCurves.size())));
		}

		bool RPLMCADShowCurvesCommand::IsOkEnabled()
		{
			bool isSourceCurvesFilePathValid = IsFilePathValid(_sourceCurvesFilePath.GetFullName());
			bool isShowedCurveFilePathValid = true;

			return isSourceCurvesFilePathValid && isShowedCurveFilePathValid;
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

			RPLM::EP::Model::EditDocument edit(GetDocument(), RSCADUIW("RPLM.CAD.UI.ShowCurves"));
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

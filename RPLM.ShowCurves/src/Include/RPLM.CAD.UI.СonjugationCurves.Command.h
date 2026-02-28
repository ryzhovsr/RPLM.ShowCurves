#pragma once

#include "RGPCommand.h"

namespace RPLM::CAD
{
	namespace UI
	{
		/// <summary>Команда сопряжения кривых</summary>
		class RPLMCADСonjugationCurvesCommand : public EP::UI::Command
		{
		public:
			/// <summary>Конструктор</summary>
		 	RPLMCADСonjugationCurvesCommand();

			/// <summary>Деструктор</summary>
			~RPLMCADСonjugationCurvesCommand();

			/// <summary>Запуск (начало выполнения) команды</summary>
			/// <param name="iParameters">Параметры инициализации команды</param>
			/// <returns>true в случае успешного запуска команды, иначе false</returns>
			virtual bool Start(EP::UI::StartCommandParameters& iParameters) override;

			/// <summary>Завершение команды</summary>
			virtual void Finish() override;

			/// <summary>Возвращает диалог команды</summary>
			/// <returns>Диалог команды</returns>
			virtual RPLM::EP::UI::ControlLayout* GetDialog() override;

			/// <summary>Получить идентификатор команды</summary>
			/// <returns>Идентификатор текущей команды</returns>
			virtual std::string GetID() override;

		private:
			/// <summary>Обрабатывает событие нажатия кнопки Ок</summary>
			void OnOK();

			/// <summary>Обрабатывает событие закрытия диалогового окна</summary>
			/// <returns>false</returns>
			bool OnCloseDialog();

			/// <summary>Обрабатывает событие изменения пути к файлу в элементах управления</summary>
			void OnFilePathChanged();

			/// <summary>Обрабатывает событие нажатия на кнопку сохранения сопряжённой кривой</summary>
			/// <param name="iControl">Кнопка, от которой пришло событие</param>
			void OnSaveConjugatedCurveInFilePressed(EP::UI::ButtonControl& iControl);

			/// <summary>Делает проверку на доступность кнопки Ок</summary>
			/// <returns>true в случае, если кнопка доступна, иначе false</returns>
			bool IsOkEnabled();

			/// <summary>Проверяет корректность пути к файлу</summary>
			/// <param name="iFilePath">Путь к файлу</param>
			/// <returns>true в случае, если путь корректен, иначе false</returns>
			bool IsFilePathValid(const Base::Framework::String& iFilePath);

			/// <summary>Отображает кривую на сцене</summary>
			/// <param name="iCurve">Кривая</param>
			/// <returns>Результат операции</returns>
			RGK::Result DrawCurve(const RGK::NURBSCurve& iCurve) const;		
		
			// Диалоговое окно
			EP::UI::ControlLayout _dialog;

			// Элемент управления для выбора файла с исходными кривыми
			EP::UI::FileNameControl _sourceCurvesFilePath;

			// Чекбокс для отображения исходных кривых на сцене
			EP::UI::ButtonControl _showSourceCurves;

			// Степень кривой
			EP::UI::EditControl _curveDegree;
			// Элемент управления для выбора файла с контрольными точками
			EP::UI::FileNameControl _controlPointsFilePath;
			// Элемент управления для выбора файла с узловыми вектором
			EP::UI::FileNameControl _knotsFilePath;

			// Чекбокс для фиксации начала кривой
			EP::UI::ButtonControl _fixBeginningCurve;
			// Чекбокс для фиксации конца кривой
			EP::UI::ButtonControl _fixEndCurve;

			// Чекбокс для сохранения сопряжённой в файл
			EP::UI::ButtonControl _saveConjugatedCurveInFile;
			// Элемент управления для выбора пути до файла, куда будут записаны данные сопряжённой кривой
			EP::UI::FileNameControl _conjugatedCurveFilePath;
		};
	}
}

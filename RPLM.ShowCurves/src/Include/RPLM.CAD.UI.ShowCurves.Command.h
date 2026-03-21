#pragma once

#include "RGPCommand.h"

namespace RPLM::CAD
{
	namespace UI
	{
		/// <summary>Команда для отображения кривых из файла</summary>
		class RPLMCADShowCurvesCommand : public EP::UI::Command
		{
		public:
			/// <summary>Конструктор</summary>
		 	RPLMCADShowCurvesCommand();

			/// <summary>Деструктор</summary>
			~RPLMCADShowCurvesCommand();

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
			// Элемент управления для отображения кол-ва кривых из файла 
			EP::UI::EditControl _numberOfCurvesFromFile;
			// Кривые, считанные из файла
			RGK::NURBSCurveVector _sourceCurves;
		};
	}
}

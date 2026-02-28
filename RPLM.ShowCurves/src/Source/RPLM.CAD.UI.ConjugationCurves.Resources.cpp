#include "RPLM.CAD.UI.СonjugationCurves.Command.h"
#include "RPLM.CAD.UI.ConjugationCurves.Resources.h"
#include "RPLM.CAD.UI.ConjugationCurves.Session.h"
#include "RPLM.UI.Widgets.MainWindow.h"
#include "RGPUIData.h"

#ifdef _DEBUG
#if defined(_MSC_VER)
#define _CATCH_LEAKS_
#endif
#ifdef _CATCH_LEAKS_
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#define DEBUG_NEW new( _NORMAL_BLOCK , __FILE__ , __LINE__ )
#define new DEBUG_NEW
#endif
#endif

namespace RPLM::Shell::UI
{
	RPLM_APP void CreateCommand(ToolContext* iContext)
	{
		if (auto uidata = RPLM::EP::UI::RGPUIData::GetUIData())
		{
			uidata->AddCommand(new RPLM::CAD::UI::RPLMCADСonjugationCurvesCommand(), iContext);
		}
	}

	// Устанавливаем доступность команды меню
	RPLM_APP void EnableCommandUpdater(ToolContext* iContext)
	{
		auto* tool = iContext->GetTool();
		tool->SetEnabled(true);
	}

	// <summary>Уведомления от пользовательского интерфеса базового приложения</summary>
	class UserInterfaceNotify final : public IInterfaceNotify
	{
	public:
		static UserInterfaceNotify* Instance()
		{
			static UserInterfaceNotify _notify;
			return &_notify;
		}

		enum class RibbonPanel
		{
			Model = 1
		};

	private:
		UserInterfaceNotify() :IInterfaceNotify(false)
		{
		}

		void InitUI(RPLM::Shell::UI::InitUIContext& iContext) override
		{
			if (auto* session = iContext.GetUISession())
			{
				auto toolID = L"CAD.Modeling.Optional.ConjugationCurves";

				// Если команда не добавлена в сессию
				if (!session->FindTool(toolID))
				{
					auto tool = new RPLM::Shell::UI::Tool(toolID, RSCADUIW("CAD.Modeling.Optional.ConjugationCurves"), toolID, &CreateCommand, &EnableCommandUpdater);
					//tool->SetEnabled(false);
					session->AddTool(tool);
				}

				if (auto* category = session->GetToolbarsDefinition().GetRibbon().GetCategory(static_cast<int>(RibbonPanel::Model)))
				{
					auto groupID = L"CAD.Modeling.Optional";

					// Если в категории нет группы
					if (!category->GetGroup(groupID))
					{
						// Добавляем группу в категорию
						auto* group = category->AddGroup(groupID, RSCADUIW("CAD.Modeling.Optional"));

						// Если в группе нет команды
						if (group && !group->GetItem(toolID, false))
						{
							// Добавляем команду в группу
							group->AddItem(RPLM::Shell::UI::ToolbarItem(toolID));
						}
					}
				}
			}
		}

		bool MainWindowClosing(const MainWindowContext& iContext) override
		{
			if (auto* uisession = iContext.GetUISession())
			{
				uisession->DisconnectInterfaceNotify(RPLM::Shell::UI::UserInterfaceNotify::Instance());
			}

			return true;
		}
	};
}

namespace RPLM::CAD::Widgets
{
	// <summary>Уведомления от сессии базового приложения</summary>
	class SessionNotify : public RPLM::Base::Framework::ISessionNotify
	{
	public:
		static SessionNotify* Instance()
		{
			static SessionNotify _notify;
			return &_notify;
		}

	private:
		SessionNotify() :ISessionNotify(false)
		{
		}

		// Параметры подключения к серверу можно передавать в командной строке приложения, в котором запускаются модули платформы
		void ReadCommandLineArguments(RPLM::Base::Framework::CommandLineArgumentsContext& iContext) override
		{
		}

		// Уведомление приложений о после запуска платформы и приложений
		void AfterInitPlatform(const RPLM::Base::Framework::AfterInitPlatformContext& iContext) override
		{
		}

		// Уведомление приложений перед завершением платформы
		void BeforeClosePlatform(const RPLM::Base::Framework::BeforeClosePlatformContext& iContext) override
		{
			// Отписываемся от уведомлений базового модуля
			if (auto* session = RPLM::Base::Framework::Session::GetSession())
			{
				session->DisconnectSessionNotifyInterface(RPLM::CAD::Widgets::SessionNotify::Instance());
			}

			RPLM::CAD::UI::Session::Instance().Destroy();
		}
	};
}

// <summary>Инициализация приложения</summary>
RPLM_APP bool InitApplication(RPLM::Base::Framework::ApplicationInitContext* context)
{
	if (auto* session = &RPLM::CAD::UI::Session::Instance())
	{
		session->Init();

		// Идентификатор модуля
		context->GetApplcation()->SetID(session->GetModuleName());
	}
	
	// Регистрация обработчика уведомлений от сессии базового приложения
	if (auto* session = RPLM::Base::Framework::Session::GetSession())
	{
		session->ConnectSessionNotifyInterface(RPLM::CAD::Widgets::SessionNotify::Instance());
	}

	// Регистрация обработчика уведомлений пользовательского интерфейса базового модуля
	if (auto* uisession = RPLM::Shell::UI::Session::GetSession())
	{
		uisession->ConnectInterfaceNotify(RPLM::Shell::UI::UserInterfaceNotify::Instance());
	}

	return true;
}
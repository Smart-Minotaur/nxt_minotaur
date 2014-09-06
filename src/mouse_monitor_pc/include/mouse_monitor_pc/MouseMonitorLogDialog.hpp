#ifndef MOUSE_MONITOR_LOG_DIALOG_H
#define MOUSE_MONITOR_LOG_DIALOG_H

#include <QDialog>
#include <QAbstractButton>
#include <QPushButton>

#include "mouse_monitor_pc/MouseMonitorNode.hpp"

#include "ui_mousemonitor_logDialog.h"

namespace minotaur
{

	class MouseMonitorLogDialog :
		public QDialog,
		public Ui::logDialog
	{
			Q_OBJECT

		private Q_SLOTS:
			void clearBtnClicked();

		public:
			MouseMonitorLogDialog(QWidget *parent = 0);
			virtual ~MouseMonitorLogDialog();

			void logRaw(MouseData data);
			void logFiltered(MouseData data);
	};

}

#endif // MOUSEMONITORLOGDIALOG_HPP

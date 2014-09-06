#ifndef MOUSE_MONITOR_MEDIAN_FILTER_DIALOG_H
#define MOUSE_MONITOR_MEDIAN_FILTER_DIALOG_H

#include <QDialog>
#include <QAbstractButton>
#include <QPushButton>
#include <QMetaType>

#include "ui_mousemonitor_medianFilterDialog.h"

#define DEFAULT_SAMPLES_NUMBER 10

namespace minotaur
{

	struct MedianFilterSettings {
		bool s1YEnabled;
		bool s1XEnabled;
		bool s2YEnabled;
		bool s2XEnabled;

		int s1Y_sampleNumbers;
		int s1X_sampleNumbers;
		int s2Y_sampleNumbers;
		int s2X_sampleNumbers;

		static MedianFilterSettings getDefaultSettings() {
			MedianFilterSettings settings;

			settings.s1YEnabled = false;
			settings.s1XEnabled = false;
			settings.s2YEnabled = false;
			settings.s2XEnabled = false;

			settings.s1Y_sampleNumbers = DEFAULT_SAMPLES_NUMBER;
			settings.s1X_sampleNumbers = DEFAULT_SAMPLES_NUMBER;
			settings.s2Y_sampleNumbers = DEFAULT_SAMPLES_NUMBER;
			settings.s2X_sampleNumbers = DEFAULT_SAMPLES_NUMBER;

			return settings;
		}
	};

	class MouseMonitorMedianFilterDialog :
		public QDialog,
		public Ui::MedianFilterDialog
	{
			Q_OBJECT

		private:
			MedianFilterSettings settings;

		private Q_SLOTS:
			void buttonBoxClicked(QAbstractButton *button);
			void okayClicked();
			void cancelClicked();
			void saveSettings();
			void displaySettings();
			
			void s1YBoxChanged(const int state);
			void s1XBoxChanged(const int state);
			void s2YBoxChanged(const int state);
			void s2XBoxChanged(const int state);

		public:
			MouseMonitorMedianFilterDialog(QWidget *parent = 0);
			virtual ~MouseMonitorMedianFilterDialog();

			MedianFilterSettings getSettings();
			bool s1Y_Enabled();
			bool s1X_Enabled();
			bool s2Y_Enabled();
			bool s2X_Enabled();

		Q_SIGNALS:
			void newMedianFilterSettings(MedianFilterSettings settings);
	};

}

Q_DECLARE_METATYPE(minotaur::MedianFilterSettings);

#endif

#include "mouse_monitor_pc/MouseMonitorMedianFilterDialog.hpp"

namespace minotaur
{

	MouseMonitorMedianFilterDialog::MouseMonitorMedianFilterDialog(QWidget *parent) : QDialog(parent)
	{
		setupUi(this);

		connect(buttonBox, SIGNAL(clicked(QAbstractButton *)), this, SLOT(buttonBoxClicked(QAbstractButton *)));
		connect(buttonBox, SIGNAL(accepted()), this, SLOT(okayClicked()));
		connect(buttonBox, SIGNAL(rejected()), this, SLOT(cancelClicked()));

		connect(s1YBox, SIGNAL(stateChanged(int)), this, SLOT(s1YBoxChanged(const int)));
		connect(s1XBox, SIGNAL(stateChanged(int)), this, SLOT(s1XBoxChanged(const int)));
		connect(s2YBox, SIGNAL(stateChanged(int)), this, SLOT(s2YBoxChanged(const int)));
		connect(s2XBox, SIGNAL(stateChanged(int)), this, SLOT(s2XBoxChanged(const int)));

		settings = MedianFilterSettings::getDefaultSettings();
		displaySettings();
	}

	MouseMonitorMedianFilterDialog::~MouseMonitorMedianFilterDialog()
	{
	}

	void MouseMonitorMedianFilterDialog::buttonBoxClicked(QAbstractButton *button)
	{
		if (buttonBox->standardButton(button) == QDialogButtonBox::Apply)
			saveSettings();
	}

	void MouseMonitorMedianFilterDialog::okayClicked()
	{
		saveSettings();

		close();
	}

	void MouseMonitorMedianFilterDialog::cancelClicked()
	{
		close();
	}

	void MouseMonitorMedianFilterDialog::saveSettings()
	{
		settings.s1Y_sampleNumbers = s1YEdit->text().toInt();
		settings.s1X_sampleNumbers = s1XEdit->text().toInt();
		settings.s2Y_sampleNumbers = s2YEdit->text().toInt();
		settings.s2X_sampleNumbers = s2XEdit->text().toInt();

		settings.s1YEnabled = s1YBox->isChecked();
		settings.s1XEnabled = s1XBox->isChecked();
		settings.s2YEnabled = s2YBox->isChecked();
		settings.s2XEnabled = s2XBox->isChecked();

		Q_EMIT newMedianFilterSettings(settings);
	}

	void MouseMonitorMedianFilterDialog::displaySettings()
	{
		s1YEdit->setText(QString::number(settings.s1Y_sampleNumbers));
		s1XEdit->setText(QString::number(settings.s1X_sampleNumbers));
		s2YEdit->setText(QString::number(settings.s2Y_sampleNumbers));
		s2XEdit->setText(QString::number(settings.s2X_sampleNumbers));

		s1YBox->setChecked(settings.s1YEnabled);
		s1XBox->setChecked(settings.s1XEnabled);
		s2YBox->setChecked(settings.s2YEnabled);
		s2XBox->setChecked(settings.s2XEnabled);
	}

	void MouseMonitorMedianFilterDialog::s1YBoxChanged(const int state)
	{
		if (state == 2)
			s1YEdit->setEnabled(true);
		else
			s1YEdit->setEnabled(false);
	}

	void MouseMonitorMedianFilterDialog::s1XBoxChanged(const int state)
	{
		if (state == 2)
			s1XEdit->setEnabled(true);
		else
			s1XEdit->setEnabled(false);
	}

	void MouseMonitorMedianFilterDialog::s2YBoxChanged(const int state)
	{
		if (state == 2)
			s2YEdit->setEnabled(true);
		else
			s2YEdit->setEnabled(false);
	}

	void MouseMonitorMedianFilterDialog::s2XBoxChanged(const int state)
	{
		if (state == 2)
			s2XEdit->setEnabled(true);
		else
			s2XEdit->setEnabled(false);
	}

	MedianFilterSettings MouseMonitorMedianFilterDialog::getSettings()
	{
		return settings;
	}

	bool MouseMonitorMedianFilterDialog::s1X_Enabled()
	{
		return s1YBox->isChecked();
	}

	bool MouseMonitorMedianFilterDialog::s1Y_Enabled()
	{
		return s1XBox->isChecked();
	}

	bool MouseMonitorMedianFilterDialog::s2X_Enabled()
	{
		return s2YBox->isChecked();
	}

	bool MouseMonitorMedianFilterDialog::s2Y_Enabled()
	{
		return s2XBox->isChecked();
	}

}

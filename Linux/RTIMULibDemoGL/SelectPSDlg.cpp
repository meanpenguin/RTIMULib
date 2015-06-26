////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "SelectPSDlg.h"
#include "RTIMUSettings.h"
#include "IMUDrivers/RTPressureDefs.h"

#include <QFormLayout>
#include <QLabel>

SelectPSDlg::SelectPSDlg(RTIMUSettings *settings, QWidget *parent)
    : QDialog(parent, Qt::WindowCloseButtonHint | Qt::WindowTitleHint)
{
    m_settings = settings;
    layoutWindow();
    setWindowTitle("Select Pressure sensor");
    connect(m_buttons, SIGNAL(accepted()), this, SLOT(onOk()));
    connect(m_buttons, SIGNAL(rejected()), this, SLOT(onCancel()));
    connect(m_selectPsensor, SIGNAL(currentIndexChanged(int)), this, SLOT(setSelectAddress(int)));
    connect(m_selectBus, SIGNAL(currentIndexChanged(int)), this, SLOT(setSelectAddress(int)));
}

SelectPSDlg::~SelectPSDlg()
{
}

void SelectPSDlg::onOk()
{
    if (m_selectBus->currentIndex() < 8) {
        // I2C
        m_settings->m_busIsI2C = true;
        m_settings->m_I2CBus = m_selectBus->currentIndex();
        m_settings->m_I2CSlaveAddress = m_selectAddress->itemData(m_selectAddress->currentIndex()).toInt();
    } else {
        // SPI
        m_settings->m_busIsI2C = false;
        m_settings->m_SPIBus = m_selectBus->currentIndex() - 8;
    }
    m_settings->m_PSType = m_selectPS->currentIndex();
    m_settings->saveSettings();

    accept();
}

void SelectPSDlg::onCancel()
{
    reject();
}

void SelectPSDlg::layoutWindow()
{
    QVBoxLayout *mainLayout;
    QFormLayout *form;

    setModal(true);

    mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(20);
    mainLayout->setContentsMargins(11, 11, 11, 11);

    form = new QFormLayout();
    mainLayout->addLayout(form);

    m_selectBus = new QComboBox();
    m_selectBus->addItem("I2C bus 0");
    m_selectBus->addItem("I2C bus 1");
    m_selectBus->addItem("I2C bus 2");
    m_selectBus->addItem("I2C bus 3");
    m_selectBus->addItem("I2C bus 4");
    m_selectBus->addItem("I2C bus 5");
    m_selectBus->addItem("I2C bus 6");
    m_selectBus->addItem("I2C bus 7");
    m_selectBus->addItem("SPI bus 0");
    m_selectBus->addItem("SPI bus 1");
    m_selectBus->addItem("SPI bus 2");
    m_selectBus->addItem("SPI bus 3");
    m_selectBus->addItem("SPI bus 4");
    m_selectBus->addItem("SPI bus 5");
    m_selectBus->addItem("SPI bus 6");
    m_selectBus->addItem("SPI bus 7");

    if (m_settings->m_busIsI2C)
        m_selectBus->setCurrentIndex(m_settings->m_I2CBus);
    else
        m_selectBus->setCurrentIndex(m_settings->m_SPIBus + 8);

    form->addRow("select bus type: ", m_selectBus);

    m_selectPS = new QComboBox();

    m_selectPS->addItem("Auto detect Psensor");
    m_selectPS->addItem("Null Psensor");
    m_selectPS->addItem("Bosch BMP180");
    m_selectPS->addItem("ST LPS25H");
    m_selectPS->addItem("Measurement Specialties MS5611");
    m_selectPS->addItem("Measurement Specialties MS5637");
    m_selectPS->addItem("Measurement Specialties MS5803");

    m_selectPS->setCurrentIndex(m_settings->m_PSType);

    form->addRow("select Pressure sensor type: ", m_selectPS);

    m_selectAddress = new QComboBox();
    setSelectAddress(m_settings->m_PsensorType, m_settings->m_I2CSlaveAddress);

    form->addRow("select I2C address type: ", m_selectAddress);

    m_buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, this);
    m_buttons->setCenterButtons(true);

    mainLayout->addWidget(m_buttons);
}

void SelectPSDlg::setSelectAddress(int)
{
    int PSType = m_selectPS->currentIndex();
    if (PSType == m_settings->m_PSType)
        setSelectAddress(PSType, m_settings->m_I2CSlaveAddress);
    else
        setSelectAddress(PSType, -1);
}

void SelectPSDlg::setSelectAddress(int PSType, int slaveAddress)
{
    m_selectAddress->clear();

    if (m_selectBus->currentIndex() < 8) {
        switch (PSType) {
        case RTPRESSURE_TYPE_BMP180:
            m_selectAddress->addItem("Standard (0x77)", BMP180_ADDRESS0);
            m_selectAddress->setCurrentIndex(0);
            break;

        case RTPRESSURE_TYPE_LPS25H:
            m_selectAddress->addItem("Standard (0x5c)", LPS25H_ADDRESS0);
            m_selectAddress->addItem("Option (0x5d)", LPS25H_ADDRESS1);
            if (slaveAddress == LPS25H_ADDRESS1)
                m_selectAddress->setCurrentIndex(1);
            else
                m_selectAddress->setCurrentIndex(0);
            break;

        case RTPRESSURE_TYPE_MS5611:
            m_selectAddress->addItem("Standard (0x76)", MS5611_ADDRESS0);
            m_selectAddress->addItem("Option (0x77)", MS5611_ADDRESS1);
            if (slaveAddress ==MS5611_ADDRESS1)
                m_selectAddress->setCurrentIndex(1);
            else
                m_selectAddress->setCurrentIndex(0);
            break;

        case RTPRESSURE_TYPE_MS5637:
            m_selectAddress->addItem("Standard (0x76)", MS5611_ADDRESS0);
            m_selectAddress->addItem("Option (0x77)", MS5611_ADDRESS1);
            if (slaveAddress == MS5611_ADDRESS1)
                m_selectAddress->setCurrentIndex(1);
            else
                m_selectAddress->setCurrentIndex(0);
            break;

        case RTPRESSURE_TYPE_MS5803:
	    m_selectAddress->addItem("Standard (0x76)", MS5611_ADDRESS0);
            m_selectAddress->addItem("Option (0x77)", MS5611_ADDRESS1);
	    if (slaveAddress == MS5611_ADDRESS1)
		m_selectAddress->setCurrentIndex(1);
	    else
		m_selectAddress->setCurrentIndex(0);
	    break;

	default:
            m_selectAddress->addItem("N/A", 0);
            break;
        }
    } else {
        switch (PSType) {
        case RTPRESSURE_TYPE_MS5803:
            m_selectAddress->addItem("Standard", MS5611_ADDRESS0);
            m_selectAddress->setCurrentIndex(0);
            break;

        default:
            m_selectAddress->addItem("N/A", 0);
            break;
        }
    }
}


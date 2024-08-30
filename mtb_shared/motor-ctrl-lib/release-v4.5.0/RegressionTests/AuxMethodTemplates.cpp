/*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


#include "AuxMethods.h"
#include <assert.h>
#include <string>
#include <fstream>
#include <utility>		// std::pair

template<class T>
CDiscreteFourierTransform<T>::CDiscreteFourierTransform(vector<T>& signal)
{
    static_assert(is_same<T, float>::value || is_same<T, double>::value, "Only float and double data types are supported!");
    m_signal = signal;
}

template<class T>
CDiscreteFourierTransform<T>::~CDiscreteFourierTransform() {}

template<class T>
complex<T> CDiscreteFourierTransform<T>::ExecuteAt(uint32_t k)
{
    const T Kw0 = static_cast<T>(k) * 2.0f * static_cast<T>(PI) / m_signal.size();
    complex<T> result = 0.0f;
    for (uint32_t index = 0; index < m_signal.size(); ++index)
    {
        T n = static_cast<T>(index);
        result += m_signal[index] * (cos(Kw0 * n) - sin(Kw0 * n) * 1.0if);
    }
    result /= static_cast<T>(m_signal.size());
    return result;
}

template<class T>
vector<complex<T>> CDiscreteFourierTransform<T>::ExecuteAll()
{
    m_result.clear();
    for (uint32_t index = 0; index < static_cast<uint32_t>(m_signal.size()) >> 1U; ++index)
    {
        m_result.push_back(ExecuteAt(index));
    }
    return m_result;
}

template<class T>
CCompareData<T>::CCompareData(vector<T>& left, vector<T>& right)
{
    static_assert(is_same<T, float>::value || is_same<T, double>::value, "Only float and double data types are supported!");
    assert(left.size() == right.size());
    m_left = left;
    m_right = right;
    m_abs_diff_max = 0.0f;
    m_abs_diff_sq_sum = 0.0f;
}

template<class T>
CCompareData<T>::CCompareData(vector<T>& left, const T right)
{
    static_assert(is_same<T, float>::value || is_same<T, double>::value, "Only float and double data types are supported!");
    m_left = left;
    m_right.assign(left.size(), right);
    m_abs_diff_max = 0.0f;
    m_abs_diff_sq_sum = 0.0f;
}

template<class T>
CCompareData<T>::~CCompareData() {};

template<class T>
void CCompareData<T>::Execute()
{
    m_abs_diff_max = 0.0f;
    m_abs_diff_sq_sum = 0.0f;
    for (uint32_t index = 0; index < m_left.size(); ++index)
    {
        T abs_diff = abs(m_left[index] - m_right[index]);
        m_abs_diff_max = max(abs_diff, m_abs_diff_max);
        m_abs_diff_sq_sum += pow(abs_diff, 2.0f);
    }
    m_abs_diff_norm = sqrt(m_abs_diff_sq_sum / m_left.size());
}

template<class T>
T CCompareData<T>::GetAbsDiffMax()
{
    return m_abs_diff_max;
}

template<class T>
T CCompareData<T>::GetAbsDiffNorm()
{
    return m_abs_diff_norm;
}

#ifdef _DEBUG	// Export debug data only in debug mode
template<class T>
void ExportData::ExportDataToCsvFile(string filename, DATA_SET_t<T> data)
{
    // Ensuring all data columns have the same size
    uint32_t data_size = data[0].second.size();
    for (uint32_t column = 1; column < data.size(); ++column)
    {
        assert(data[column].second.size() == data_size); // all columns should have the same size
    }

    // Open file
    ofstream myFile("../RegressionTests/ExportedData/" + filename);

    // Write column labels:
    for (uint32_t column = 0; column < data.size(); ++column)
    {
        myFile << data[column].first;
        myFile << ((column != data.size() - 1) ? ",\t" : ""); // No comma at the end of line
    }
    myFile << "\n";

    // Write column data:
    for (uint32_t row = 0; row < data_size; ++row)
    {
        for (uint32_t column = 0; column < data.size(); ++column)
        {
            myFile << data[column].second[row];
            myFile << ((column != data.size() - 1) ? ",\t" : "");
        }
        myFile << "\n";
    }

    // Close file
    myFile.close();
}
#else
template<class T>
void ExportData::ExportDataToCsvFile(string filename, DATA_SET_t<T> data) {}
#endif

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


#pragma once

#include "../ThirdPartyLib/cpputest/include/CppUTest/TestHarness.h"
#include <math.h>
#include <complex>
#include <vector>
#ifdef __cplusplus
extern "C"
{
#endif
#include "../OperationalCode/General.h"
#ifdef __cplusplus
}
#endif

using namespace std;

#define	DOUBLES_EQUAL_PERC(expected, actual, percent)	DOUBLES_EQUAL((expected), (actual), ABS((expected)*(percent)))
#define ANGLE_EQUAL(expected, actual, threshold)		DOUBLES_EQUAL(0.0f, Wrap2Pi((expected)-(actual)), (threshold))

template<class T>
class CDiscreteFourierTransform
{
public:
    CDiscreteFourierTransform(vector<T>& signal);
    ~CDiscreteFourierTransform();
    complex<T> ExecuteAt(uint32_t k);
    vector<complex<T>> ExecuteAll();
private:
    vector<T> m_signal;
    vector<complex<T>> m_result;
};

template<class T>
class CCompareData
{
public:
    CCompareData(vector<T>& left, vector<T>& right);
    CCompareData(vector<T>& left, const T right);
    ~CCompareData();
    void Execute();
    T GetAbsDiffMax(); // Absolute difference max
    T GetAbsDiffNorm(); // Absolute difference norm
private:
    vector<T> m_left;
    vector<T> m_right;
    T m_abs_diff_max;
    T m_abs_diff_sq_sum;
    T m_abs_diff_norm;
};

class CReferenceFrame
{
public:
    CReferenceFrame();
    CReferenceFrame(float theta, AB_t ab);
    CReferenceFrame(float theta, QD_t qd);
    CReferenceFrame(float theta, POLAR_t polar);
    ~CReferenceFrame();

    void SetAB(float theta, float ab_a, float ab_b);
    void SetAB(float theta, AB_t ab);

    void SetQD(float theta, float qd_q, float qd_d);
    void SetQD(float theta, QD_t qd);

    void SetPolar(float theta, float polar_rad, float polar_theta);
    void SetPolar(float theta, POLAR_t polar);

    QD_t GetQD();
    AB_t GetAB();
    POLAR_t GetPolar();
private:
    float m_theta;
    PARK_t m_park;
    AB_t m_ab;
    QD_t m_qd;
    POLAR_t m_polar;
};

class CDiscreteInteg
{
public:
    enum class EMethod
    {
        Forward_Euler = 0,
        Backward_Euler,
        Bilinear,
        Max
    };

    CDiscreteInteg(EMethod method);
    ~CDiscreteInteg();

    void SetMethod(EMethod method);
    EMethod GetMethod();

    void Reset(float integ);
    float Run(float input);

    float m_integ;
    float m_prev_input;

private:
    float (CDiscreteInteg::* RunPtr)(float input);
    float RunForwardEuler(float input);
    float RunBackwardEuler(float input);
    float RunBilinear(float input);

    EMethod m_method;
};

// Using bisection method, this class finds the roots of any equation in [lim.min,lim.max] interval 
class CBisectionRoots
{
public:
    using EquationFunc = float (*)(float);

    CBisectionRoots(EquationFunc equation, MINMAX_t lim, float error_max);
    ~CBisectionRoots();

    float Find();

    uint32_t m_try_count;
    static const uint32_t Max_Tries = (1U << 10U);
    bool m_fault;

private:
    EquationFunc m_equation;
    MINMAX_t m_lim;
    float m_error_max;
};

namespace ExportData
{
    // Exporting unit test data to csv files for usage by Excel, MATLAB, etc.
    // 
    // Export folder is "../RegressionTests/ExportedData/"
    // 
    // Example usage (C++ side):
    // vector<float> time, signal;
    // ...
    // ExportData::DATA_SET_t<float> waveform = {
    //	ExportData::DATA_PAIR_t<float>("Time [msec]",time),
    //	ExportData::DATA_PAIR_t<float>("Signal [V]",signal),
    // }
    // ExportDataToCsvFile("FileName.csv", waveform);
    //
    // Example usage (MATLAB side):
    // data = readmatrix("FileName.csv","NumHeaderLines",1);
    // or
    // data = readtable("FileName.csv");

    template<class T>
    using DATA_PAIR_t = pair<string, vector<T>>;

    template<class T>
    using DATA_SET_t = vector<DATA_PAIR_t<T>>;

    template<class T>
    void ExportDataToCsvFile(string filename, DATA_SET_t<T> data);

    vector<float> CollectQ(vector<QD_t>& input);
    vector<float> CollectD(vector<QD_t>& input);
    vector<float> CollectA(vector<AB_t>& input);
    vector<float> CollectB(vector<AB_t>& input);
}

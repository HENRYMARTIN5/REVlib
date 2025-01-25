/*
 * Copyright (c) 2024 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package com.revrobotics.spark.config;

import com.revrobotics.jni.CANSparkJNI;

public class SignalsConfigAccessor {
  private final long sparkHandle;

  protected SignalsConfigAccessor(long sparkHandle) {
    this.sparkHandle = sparkHandle;
  }

  public int getAppliedOutputPeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus0Period.value);
  }

  public boolean getAppliedOutputAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus0.value)
        != 0;
  }

  public int getBusVoltagePeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus0Period.value);
  }

  public boolean getBusVoltageAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus0.value)
        != 0;
  }

  public int getOutputCurrentPeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus0Period.value);
  }

  public boolean getOutputCurrentAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus0.value)
        != 0;
  }

  public int getMotorTemperaturePeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus0Period.value);
  }

  public boolean getMotorTemperatureAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus0.value)
        != 0;
  }

  public int getLimitsPeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus0Period.value);
  }

  public boolean getLimitsAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus0.value)
        != 0;
  }

  public int getFaultsPeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus1Period.value);
  }

  public boolean getFaultsAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus1.value)
        != 0;
  }

  public int getWarningsPeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus1Period.value);
  }

  public boolean getWarningsAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus1.value)
        != 0;
  }

  public int getPrimaryEncoderVelocityPeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus2Period.value);
  }

  public boolean getPrimaryEncoderVelocityAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus2.value)
        != 0;
  }

  public int getPrimaryEncoderPositionPeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus2Period.value);
  }

  public boolean getPrimaryEncoderPositionAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus2.value)
        != 0;
  }

  public int getAnalogVoltagePeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus3Period.value);
  }

  public boolean getAnalogVoltageAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus3.value)
        != 0;
  }

  public int getAnalogVelocityPeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus3Period.value);
  }

  public boolean getAnalogVelocityAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus3.value)
        != 0;
  }

  public int getAnalogPositionPeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus3Period.value);
  }

  public boolean getAnalogPositionAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus3.value)
        != 0;
  }

  public int getExternalOrAltEncoderVelocityPeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus4Period.value);
  }

  public boolean getExternalOrAltEncoderVelocityAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus4.value)
        != 0;
  }

  public int getExternalOrAltEncoderPositionPeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus4Period.value);
  }

  public boolean getExternalOrAltEncoderPositionAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus4.value)
        != 0;
  }

  public int getAbsoluteEncoderVelocityPeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus5Period.value);
  }

  public boolean getAbsoluteEncoderVelocityAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus5.value)
        != 0;
  }

  public int getAbsoluteEncoderPositionPeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus5Period.value);
  }

  public boolean getAbsoluteEncoderPositionAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus5.value)
        != 0;
  }

  public int getIAccumulationPeriodMs() {
    return CANSparkJNI.c_Spark_GetParameterUint32(sparkHandle, SparkParameter.kStatus7Period.value);
  }

  public boolean getIAccumulationAlwaysOn() {
    return CANSparkJNI.c_Spark_GetParameterUint32(
            sparkHandle, SparkParameter.kForceEnableStatus7.value)
        != 0;
  }
}

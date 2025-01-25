/*
 * Copyright (c) 2018-2024 REV Robotics
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

package com.revrobotics.spark;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANSparkJNI;

// package-private to the revrobotics package
public class SparkFlexExternalEncoder implements RelativeEncoder {
  // package-private to the revrobotics package
  final SparkFlex sparkFlex;

  // package-private to the revrobotics package
  SparkFlexExternalEncoder(SparkFlex sparkFlex) {
    this.sparkFlex = sparkFlex;

    // If we ever add additional entries to the ExternalEncoderType enum, we need to use the
    // encoderType param.

    // let the driver check if sim is running and create sim if necessary
    // alternate and external encoders are handled through the same endpoints
    // and are differentiated on the driver side
    CANSparkJNI.c_Spark_CreateAlternateEncoderSim(sparkFlex.sparkHandle);
  }

  public double getPosition() {
    sparkFlex.throwIfClosed();
    return CANSparkJNI.c_Spark_GetAltEncoderPosition(sparkFlex.sparkHandle);
  }

  public double getVelocity() {
    sparkFlex.throwIfClosed();
    return CANSparkJNI.c_Spark_GetAltEncoderVelocity(sparkFlex.sparkHandle);
  }

  public REVLibError setPosition(double position) {
    sparkFlex.throwIfClosed();
    return REVLibError.fromInt(
        CANSparkJNI.c_Spark_SetAltEncoderPosition(sparkFlex.sparkHandle, (float) position));
  }
}

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

enum SparkParameter {
  kCanID(0),
  kInputMode(1),
  kMotorType(2),
  kCommAdvance(3),
  kSensorType(4),
  kCtrlType(5),
  kIdleMode(6),
  kInputDeadband(7),
  kLegacyFeedbackSensorPID0(8),
  kClosedLoopControlSensor(9),
  kPolePairs(10),
  kCurrentChop(11),
  kCurrentChopCycles(12),
  kP_0(13),
  kI_0(14),
  kD_0(15),
  kF_0(16),
  kIZone_0(17),
  kDFilter_0(18),
  kOutputMin_0(19),
  kOutputMax_0(20),
  kP_1(21),
  kI_1(22),
  kD_1(23),
  kF_1(24),
  kIZone_1(25),
  kDFilter_1(26),
  kOutputMin_1(27),
  kOutputMax_1(28),
  kP_2(29),
  kI_2(30),
  kD_2(31),
  kF_2(32),
  kIZone_2(33),
  kDFilter_2(34),
  kOutputMin_2(35),
  kOutputMax_2(36),
  kP_3(37),
  kI_3(38),
  kD_3(39),
  kF_3(40),
  kIZone_3(41),
  kDFilter_3(42),
  kOutputMin_3(43),
  kOutputMax_3(44),
  kInverted(45),
  kOutputRatio(46),
  kSerialNumberLow(47),
  kSerialNumberMid(48),
  kSerialNumberHigh(49),
  kLimitSwitchFwdPolarity(50),
  kLimitSwitchRevPolarity(51),
  kHardLimitFwdEn(52),
  kHardLimitRevEn(53),
  kSoftLimitFwdEn(54),
  kSoftLimitRevEn(55),
  kRampRate(56),
  kFollowerID(57),
  kFollowerConfig(58),
  kSmartCurrentStallLimit(59),
  kSmartCurrentFreeLimit(60),
  kSmartCurrentConfig(61),
  kSmartCurrentReserved(62),
  kMotorKv(63),
  kMotorR(64),
  kMotorL(65),
  kMotorRsvd1(66),
  kMotorRsvd2(67),
  kMotorRsvd3(68),
  kEncoderCountsPerRev(69),
  kEncoderAverageDepth(70),
  kEncoderSampleDelta(71),
  kEncoderInverted(72),
  kEncoderRsvd1(73),
  kVoltageCompMode(74),
  kCompensatedNominalVoltage(75),
  kSmartMotionMaxVelocity_0(76),
  kSmartMotionMaxAccel_0(77),
  kSmartMotionMinVelOutput_0(78),
  kSmartMotionAllowedClosedLoopError_0(79),
  kSmartMotionAccelStrategy_0(80),
  kSmartMotionMaxVelocity_1(81),
  kSmartMotionMaxAccel_1(82),
  kSmartMotionMinVelOutput_1(83),
  kSmartMotionAllowedClosedLoopError_1(84),
  kSmartMotionAccelStrategy_1(85),
  kSmartMotionMaxVelocity_2(86),
  kSmartMotionMaxAccel_2(87),
  kSmartMotionMinVelOutput_2(88),
  kSmartMotionAllowedClosedLoopError_2(89),
  kSmartMotionAccelStrategy_2(90),
  kSmartMotionMaxVelocity_3(91),
  kSmartMotionMaxAccel_3(92),
  kSmartMotionMinVelOutput_3(93),
  kSmartMotionAllowedClosedLoopError_3(94),
  kSmartMotionAccelStrategy_3(95),
  kIMaxAccum_0(96),
  kSlot3Placeholder1_0(97),
  kSlot3Placeholder2_0(98),
  kSlot3Placeholder3_0(99),
  kIMaxAccum_1(100),
  kSlot3Placeholder1_1(101),
  kSlot3Placeholder2_1(102),
  kSlot3Placeholder3_1(103),
  kIMaxAccum_2(104),
  kSlot3Placeholder1_2(105),
  kSlot3Placeholder2_2(106),
  kSlot3Placeholder3_2(107),
  kIMaxAccum_3(108),
  kSlot3Placeholder1_3(109),
  kSlot3Placeholder2_3(110),
  kSlot3Placeholder3_3(111),
  kPositionConversionFactor(112),
  kVelocityConversionFactor(113),
  kClosedLoopRampRate(114),
  kSoftLimitFwd(115),
  kSoftLimitRev(116),
  kSoftLimitRsvd0(117),
  kSoftLimitRsvd1(118),
  kAnalogRevPerVolt(119),
  kAnalogRotationsPerVoltSec(120),
  kAnalogAverageDepth(121),
  kAnalogSensorMode(122),
  kAnalogInverted(123),
  kAnalogSampleDelta(124),
  kAnalogRsvd0(125),
  kAnalogRsvd1(126),
  kDataPortConfig(127),
  kAltEncoderCountsPerRev(128),
  kAltEncoderAverageDepth(129),
  kAltEncoderSampleDelta(130),
  kAltEncoderInverted(131),
  kAltEncodePositionFactor(132),
  kAltEncoderVelocityFactor(133),
  kAltEncoderRsvd0(134),
  kAltEncoderRsvd1(135),
  kHallSensorSampleRate(136),
  kHallSensorAverageDepth(137),
  kNumParameters(138),
  kDutyCyclePositionFactor(139),
  kDutyCycleVelocityFactor(140),
  kDutyCycleInverted(141),
  kDutyCycleSensorMode(142),
  kDutyCycleAverageDepth(143),
  kDutyCycleSampleDelta(144),
  kDutyCycleOffsetv1p6p2(145),
  kDutyCycleRsvd0(146),
  kDutyCycleRsvd1(147),
  kDutyCycleRsvd2(148),
  kPositionPIDWrapEnable(149),
  kPositionPIDMinInput(150),
  kPositionPIDMaxInput(151),
  kDutyCycleZeroCentered(152),
  kDutyCyclePrescaler(153),
  kDutyCycleOffset(154),
  kProductId(155),
  kDeviceMajorVersion(156),
  kDeviceMinorVersion(157),
  kStatus0Period(158),
  kStatus1Period(159),
  kStatus2Period(160),
  kStatus3Period(161),
  kStatus4Period(162),
  kStatus5Period(163),
  kStatus6Period(164),
  kStatus7Period(165),
  kMAXMotionMaxVelocity_0(166),
  kMAXMotionMaxAccel_0(167),
  kMAXMotionMaxJerk_0(168),
  kMAXMotionAllowedClosedLoopError_0(169),
  kMAXMotionPositionMode_0(170),
  kMAXMotionMaxVelocity_1(171),
  kMAXMotionMaxAccel_1(172),
  kMAXMotionMaxJerk_1(173),
  kMAXMotionAllowedClosedLoopError_1(174),
  kMAXMotionPositionMode_1(175),
  kMAXMotionMaxVelocity_2(176),
  kMAXMotionMaxAccel_2(177),
  kMAXMotionMaxJerk_2(178),
  kMAXMotionAllowedClosedLoopError_2(179),
  kMAXMotionPositionMode_2(180),
  kMAXMotionMaxVelocity_3(181),
  kMAXMotionMaxAccel_3(182),
  kMAXMotionMaxJerk_3(183),
  kMAXMotionAllowedClosedLoopError_3(184),
  kMAXMotionPositionMode_3(185),
  kForceEnableStatus0(186),
  kForceEnableStatus1(187),
  kForceEnableStatus2(188),
  kForceEnableStatus3(189),
  kForceEnableStatus4(190),
  kForceEnableStatus5(191),
  kForceEnableStatus6(192),
  kForceEnableStatus7(193),
  kFollowerModeLeaderId(194),
  kFollowerModeIsInverted(195),
  kDutyCycleEncoderStartPulseUs(196),
  kDutyCycleEncoderEndPulseUs(197);

  @SuppressWarnings("MemberName")
  public final int value;

  SparkParameter(int value) {
    this.value = value;
  }
}

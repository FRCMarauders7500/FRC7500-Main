// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;

/** IO implementation for NavX */
public class GyroIONavX implements GyroIO {
  private final AHRS m_gyro;

  public GyroIONavX() {
    this.m_gyro = new AHRS();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = m_gyro.isConnected();
    inputs.headingDeg = -m_gyro.getAngle();
    inputs.rollDeg = m_gyro.getRoll();
    inputs.headingRateDPS = m_gyro.getRate();
    inputs.headingAdjustment = m_gyro.getAngleAdjustment();
  }

  @Override
  public void resetHeading() {
    // m_gyro.setAngleAdjustment
  }

  @Override
  public void setHeading(Rotation2d heading) {
    m_gyro.setAngleAdjustment(0);
    m_gyro.setAngleAdjustment((-heading.getDegrees()) - m_gyro.getAngle());
  }
}

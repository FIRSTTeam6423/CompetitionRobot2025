// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package lib.drivers;

public class CanDeviceId {
  private final int id;
  private final String busName;

  public CanDeviceId(int id, String busName) {
    this.id = id;
    this.busName = busName;
  }

  public int getId() {
    return id;
  }

  public String getBusName() {
    return busName;
  }
}

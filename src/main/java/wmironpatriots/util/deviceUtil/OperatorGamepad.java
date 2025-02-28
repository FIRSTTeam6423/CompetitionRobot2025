// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.util.deviceUtil;

import java.util.function.Supplier;
import wmironpatriots.Constants.ReefTarget;

public class OperatorGamepad implements Supplier<ReefTarget> {

  public OperatorGamepad() {}

  @Override
  public ReefTarget get() {
    return ReefTarget.L1;
  }
}

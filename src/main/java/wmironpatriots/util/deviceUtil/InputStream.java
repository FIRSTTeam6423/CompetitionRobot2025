// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.util.deviceUtil;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

/** Inspired by 1155's InputStream class A functional interface for modifying double suppliers */
@FunctionalInterface
public interface InputStream extends DoubleSupplier {
  /** Create a new input stream from a previously existing stream */
  public static InputStream of(DoubleSupplier stream) {
    return stream::getAsDouble;
  }

  public static InputStream hypot(InputStream y, InputStream x) {
    return () -> Math.hypot(x.get(), y.get());
  }

  public static InputStream arcTan(InputStream y, InputStream x) {
    return () -> Math.atan2(y.get(), x.get());
  }

  public default double get() {
    return getAsDouble();
  }

  /** Maps stream value by an operator */
  public default InputStream map(DoubleUnaryOperator operator) {
    return () -> operator.applyAsDouble(get());
  }

  /** Scale stream value by factor */
  public default InputStream scale(double factor) {
    return () -> get() * factor;
  }

  /** Scale stream value by factor */
  public default InputStream scale(DoubleSupplier factorSupplier) {
    return () -> get() * factorSupplier.getAsDouble();
  }

  /** Returns the stream value to the power of specified exponent while keeping its sign */
  public default InputStream signedPow(double power) {
    double value = get();
    return () -> Math.pow(value, power) * Math.signum(value);
  }

  /**
   * Deadbands stream's value within specified range around 0 and clamps value to specified distance
   * from 0
   */
  public default InputStream deadband(double deadband, double max) {
    double value = get();
    return () -> Math.abs(value) > deadband ? Math.max(-max, Math.min(value, max)) : 0.0;
  }
}

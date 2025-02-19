// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package monologue;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.struct.StructSerializable;

class TypeChecker {
  private static final Class<?>[] LITERAL_TYPES = {
    boolean.class,
    int.class,
    long.class,
    float.class,
    double.class,
    Boolean.class,
    Integer.class,
    Long.class,
    Float.class,
    Double.class,
    String.class,
    StructSerializable.class
  };

  private static final Class<?>[] EXTENDABLE_TYPES = {Sendable.class};

  static boolean isValidLiteralType(Class<?> type) {
    for (Class<?> literalType : LITERAL_TYPES) {
      if (literalType.isAssignableFrom(type)) {
        return true;
      }
    }
    return false;
  }

  static boolean isValidExtendableType(Class<?> type) {
    for (Class<?> extendableType : EXTENDABLE_TYPES) {
      if (extendableType.isAssignableFrom(type)) {
        return true;
      }
    }
    return false;
  }

  static boolean isValidType(Class<?> type) {
    Class<?> et = type.isArray() ? type.getComponentType() : type;
    return isValidLiteralType(et) || isValidExtendableType(et);
  }

  static enum TypeRoutes {
    DOUBLE,
    INTEGER,
    BOOLEAN,
    OBJECT
  }
}

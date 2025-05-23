// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package lib.utils;

import java.lang.reflect.InvocationTargetException;
import monologue.Monologue;

// TODO make this less stupid :)

/** Utility class for logging /w {@link Monologue} */
public class LogUtils {
  /**
   * Automatically logs all fields of an object of a record with monologue
   *
   * <p>Supports {@link Boolean}, {@link Double}, {@link Integer}
   *
   * @param loggableState desired record object to log
   */
  public static void logRecord(Record loggableRecord) {
    logRecord("", loggableRecord);
  }

  /**
   * Automatically logs all fields of an object of a record with monologue
   *
   * <p>Supports {@link Boolean}, {@link Double}, {@link Integer}
   *
   * @param prefix path prefix to log to
   * @param loggableState desired record object to log
   */
  public static void logRecord(String prefix, Record loggableState) {
    var components = loggableState.getClass().getRecordComponents();

    prefix = "Logged/" + prefix + "/";

    for (var component : components) {
      try {
        var obj = component.getAccessor().invoke(loggableState);

        if (obj instanceof Boolean) {
          Monologue.log(prefix + component.getName(), (Boolean) obj);
        } else if (obj instanceof Double) {
          Monologue.log(prefix + component.getName(), (Double) obj);
        } else if (obj instanceof Integer) {
          Monologue.log(prefix + component.getName(), (Integer) obj);
        } else {
          System.out.println(
              "LogUtils Error! Cannot log the " + obj.getClass().getTypeName() + " type");
        }
      } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
        e.printStackTrace();
      }
    }
  }
}
// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package wmironpatriots.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj.Timer;
import wmironpatriots.Constants;

/**
 * Inspired by FRC 6328's 2024 odometry thread
 * 
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class TalonOdoThread extends Thread {
  private final Lock signalsLock =
      new ReentrantLock(); // Prevents conflicts when registering signals
  private BaseStatusSignal[] signals = new BaseStatusSignal[0];
  private final List<Queue<Double>> queues = new ArrayList<>();
  private boolean isCANFD = false;

  private static TalonOdoThread instance = null;

  public static TalonOdoThread getInstance() {
    if (instance == null) {
      instance = new TalonOdoThread();
    }
    return instance;
  }

  private TalonOdoThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
    start();
  }

  public Queue<Double> registerSignal(ParentDevice device, BaseStatusSignal signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    Swerve.odoLock.lock();
    try {
      isCANFD = CANBus.isNetworkFD(device.getNetwork());
      BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
      System.arraycopy(signals, 0, newSignals, 0, signals.length);
      newSignals[signals.length] = signal;
      signals = newSignals;
      queues.add(queue);
    } finally {
      signalsLock.unlock();
      Swerve.odoLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      signalsLock.lock();
      try {
        if (isCANFD) {
          BaseStatusSignal.waitForAll(Constants.TICK_SPEED, signals);
        } else {
          Thread.sleep((long) (1000.0 / Swerve.ODO_FREQ)); // * ODO FREQ goes here
          if (signals.length > 0) BaseStatusSignal.refreshAll(signals);
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        signalsLock.unlock();
      }
      double fpgaTimestamp = Timer.getFPGATimestamp();

      // Save new data to queues
      Swerve.odoLock.lock();
      try {
        for (int i = 0; i < signals.length; i++) {
          queues.get(i).offer(signals[i].getValueAsDouble());
        }
        Swerve.timestampQueue.offer(fpgaTimestamp);
      } finally {
        Swerve.odoLock.unlock();
      }
    }
  }
}
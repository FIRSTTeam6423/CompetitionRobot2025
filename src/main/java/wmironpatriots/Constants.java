package wmironpatriots;

import com.ctre.phoenix6.CANBus;

public class Constants {
    /** static class containing all device ids */
    public static class MATRIXID {
        // * CANIVORE LOOP
        public static final CANBus CANCHAN = new CANBus("can-chan"); // :3
        public static final int PIGEON = 0;
        public static final int BL_PIVOT = 1;
        public static final int BL_DRIVE = 2;
        public static final int FL_PIVOT = 3;
        public static final int FL_DRIVE = 4;
        public static final int FR_PIVOT = 5;
        public static final int FR_DRIVE = 6;
        public static final int BR_PIVOT = 7;
        public static final int BR_DRIVE = 8;
        public static final int BL_CANCODER = 9;
        public static final int FL_CANCODER = 10;
        public static final int FR_CANCODER = 11;
        public static final int BR_CANCODER = 12;
        public static final int ELEVATOR_PARENT = 14;
        public static final int ELEVATOR_CHILD = 15;

        // * RIO LOOP
        public static final CANBus RIO = new CANBus("rio");
        public static final int TAIL_ROLLER = 1;
        public static final int CHUTE_ROLLER = 2;
        public static final int TAIL_PIVOT = 13;
    }
}

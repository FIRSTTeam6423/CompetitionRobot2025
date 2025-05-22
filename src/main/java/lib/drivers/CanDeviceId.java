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

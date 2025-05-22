package lib;

public class CanDevice {
  private final int id;
  private final String busName;

  public CanDevice(int id, String busName) {
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

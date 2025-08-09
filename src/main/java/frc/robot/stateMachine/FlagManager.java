
package frc.robot.stateMachine;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;
import dev.doglog.DogLog;

public class FlagManager<T extends Enum<T>> {
  private final String loggerCategory;

  private final Set<T> checked;

  public FlagManager(String loggerCategory, Class<T> flag) {
    this.loggerCategory = loggerCategory;
    this.checked = EnumSet.noneOf(flag);
  }

  public void log() {
    DogLog.log(loggerCategory + "/Flags", checked.stream().map(Enum::name).toArray(String[]::new));
  }

  public void check(T flag) {
    checked.add(flag);
  }

  public List<T> getChecked() {
    return new ArrayList<>(checked);
  }

  public void clear() {
    checked.clear();
  }
}
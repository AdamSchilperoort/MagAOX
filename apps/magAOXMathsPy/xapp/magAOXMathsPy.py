import math
import time
import xconf

from purepyindi2 import constants, properties
from purepyindi2.messages import DefNumber, DefText

from magaox.constants import StateCodes
from magaox.indi.device import BaseConfig, XDevice


@xconf.config
class MagAOXMathsPyConfig(BaseConfig):
    """Python equivalent of the C++ magAOXMaths app."""

    myVal: str = xconf.field(default="x", help="Name of this app's input value property.")
    otherDevName: str = xconf.field(help="Name of the external INDI device providing the other value.")
    otherValName: str = xconf.field(help="Name of the external INDI property providing the other value.")
    startVal: float = xconf.field(default=0.0, help="Initial value for this app's input.")
    sleep_interval_sec: float = xconf.field(default=0.25, help="Sleep interval between loop() calls.")


class magAOXMathsPy(XDevice):
    config: MagAOXMathsPyConfig

    _other_value: float = 0.0
    _warned_missing_other: bool = False

    def setup(self):
        fsm = properties.TextVector(name="fsm")
        fsm.add_element(DefText(name="state", _value=StateCodes.INITIALIZED.name))
        self.add_property(fsm)

        my_val = properties.NumberVector(
            name=self.config.myVal,
            perm=constants.PropertyPerm.READ_WRITE,
        )
        my_val.add_element(
            DefNumber(
                name="value",
                label=self.config.myVal,
                format="%f",
                min=-1e50,
                max=1e50,
                step=1.0,
                _value=self.config.startVal,
            )
        )
        self.add_property(my_val, callback=self.handle_my_val)

        maths = properties.NumberVector(
            name="maths",
            perm=constants.PropertyPerm.READ_ONLY,
        )
        maths.add_element(DefNumber(name="value", label="value", format="%f", min=-1e50, max=1e50, step=1.0, _value=0.0))
        maths.add_element(DefNumber(name="sqr", label="sqr", format="%f", min=-1e50, max=1e50, step=1.0, _value=0.0))
        maths.add_element(DefNumber(name="sqrt", label="sqrt", format="%f", min=-1e50, max=1e50, step=1.0, _value=0.0))
        maths.add_element(DefNumber(name="abs", label="abs", format="%f", min=-1e50, max=1e50, step=1.0, _value=0.0))
        maths.add_element(DefNumber(name="prod", label="prod", format="%f", min=-1e50, max=1e50, step=1.0, _value=0.0))
        self.add_property(maths)

        other_val = properties.NumberVector(
            name="other_val",
            perm=constants.PropertyPerm.READ_WRITE,
        )
        other_val.add_element(
            DefNumber(
                name="current",
                label="Current external value",
                format="%f",
                min=-1e50,
                max=1e50,
                step=1.0,
                _value=0.0,
            )
        )
        other_val.add_element(
            DefNumber(
                name="target",
                label="Target external value",
                format="%f",
                min=-1e50,
                max=1e50,
                step=1.0,
                _value=0.0,
            )
        )
        self.add_property(other_val, callback=self.handle_set_other_val)

        while self.client.status is not constants.ConnectionStatus.CONNECTED:
            self.log.info("Waiting for INDI client connection...")
            time.sleep(1)

        self.client.get_properties(f"{self.config.otherDevName}.{self.config.otherValName}")
        self._refresh_other_value_from_client(force_update=False)
        self._update_vals()

        self.properties["fsm"]["state"] = StateCodes.READY.name
        self.update_property(self.properties["fsm"])

    def _other_value_indi_id(self):
        return f"{self.config.otherDevName}.{self.config.otherValName}.value"

    def _coerce_float(self, value):
        if hasattr(value, "value"):
            value = value.value
        return float(value)

    def _refresh_other_value_from_client(self, force_update):
        try:
            new_val = self._coerce_float(self.client[self._other_value_indi_id()])
        except Exception:
            if not self._warned_missing_other:
                self.log.warning(
                    "Could not read external value %s; waiting for property to appear.",
                    self._other_value_indi_id(),
                )
                self._warned_missing_other = True
            return

        self._warned_missing_other = False
        if force_update or new_val != self._other_value:
            self._other_value = new_val
            self.properties["other_val"]["current"] = new_val
            self.update_property(self.properties["other_val"])
            self._update_vals()

    def _update_vals(self):
        v = float(self.properties[self.config.myVal]["value"])

        if v == -1:
            self.log.warning("value set to -1!")
        if v == -2:
            self.log.error("value set to -2!")
        if v == -3:
            self.log.critical("value set to -3!")
        if v == -4:
            self.log.critical("value set to -4! (alert)")
        if v == -5:
            self.log.critical("value set to -5! (emergency)")

        if v >= 0:
            sqrt_v = math.sqrt(v)
        else:
            sqrt_v = float("nan")

        self.properties["maths"]["value"] = v
        self.properties["maths"]["sqr"] = v * v
        self.properties["maths"]["sqrt"] = sqrt_v
        self.properties["maths"]["abs"] = abs(v)
        self.properties["maths"]["prod"] = v * self._other_value
        self.update_property(self.properties["maths"])

        self.log.info("set new value: %s", v)

    def _send_other_target(self, target):
        try:
            self.client[self._other_value_indi_id()] = target
        except Exception:
            self.log.exception("Failed to send target value %.6f to %s", target, self._other_value_indi_id())

    def handle_my_val(self, existing_property, new_message):
        if "value" in new_message:
            new_val = self._coerce_float(new_message["value"])
            existing_property["value"] = new_val
            self.update_property(existing_property)
            self._update_vals()
        else:
            self.update_property(existing_property)

    def handle_set_other_val(self, existing_property, new_message):
        if "target" in new_message:
            target = self._coerce_float(new_message["target"])
            existing_property["target"] = target
            self.update_property(existing_property)
            self._send_other_target(target)
            self._refresh_other_value_from_client(force_update=True)
        else:
            self.update_property(existing_property)

    def loop(self):
        self._refresh_other_value_from_client(force_update=False)

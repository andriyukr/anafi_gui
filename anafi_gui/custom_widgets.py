from typing import Tuple, Iterable, List, Callable
from PyQt6 import QtCore, QtWidgets
#from anafi_gui.droneui import UI

class TitleLabel(QtWidgets.QLabel):
    def __init__(self, layout: QtWidgets.QLayout, parent: QtWidgets.QWidget, text: str = "",
                 property_class: str = "title",
                 text_alignment: QtCore.Qt.AlignmentFlag = QtCore.Qt.AlignmentFlag.AlignCenter,
                 layout_alignment: QtCore.Qt.AlignmentFlag = (
                         QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignTop),
                 layout_strech: int = 0):
        super().__init__(parent)
        self.setText(text)
        self.setAlignment(text_alignment)
        self.setWordWrap(True)
        self.setProperty("class", property_class)
        layout.addWidget(self, layout_strech, layout_alignment)


class EnhancedFloatQSlider:
    def __init__(self, parent: QtWidgets.QGroupBox, minimum: float, maximum: float, initial_value: float = None,
                 step: float = 1, prefix: str = "", suffix: str = "", minimum_width: int = 150,
                 pairs: List['SliderGroupBox'] = None):

        initial_value = initial_value or minimum
        self.step = step

        # value label (QDoubleSpinBox)
        self.valueLabel = QtWidgets.QDoubleSpinBox(parent)
        self.valueLabel.setRange(minimum, maximum)
        self.valueLabel.setValue(initial_value)
        self.valueLabel.setSuffix(suffix)
        self.valueLabel.setPrefix(prefix)
        self.valueLabel.setSingleStep(step)
        self.valueLabel.setKeyboardTracking(False)
        self.valueLabel.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.valueLabel.setFixedHeight(25)
        self.valueLabel.setFixedWidth(130)
        self.valueLabel.setButtonSymbols(QtWidgets.QSpinBox.ButtonSymbols.PlusMinus)
        self.currentValue = initial_value
        self.valueLabel.valueChanged.connect(lambda: self.spinBoxValueChanged())
        self.valueLabel.editingFinished.connect(lambda: self.spinBoxValueChanged())

        # slider
        self.slider = QtWidgets.QSlider(parent)
        self.slider.setRange(int(minimum / step), int(maximum / step))
        self.slider.setTickInterval(15)
        self.slider.setValue(int(initial_value / step))
        self.slider.setMinimumWidth(minimum_width)
        self.slider.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.slider.valueChanged.connect(lambda: self.valueLabel.setValue(self.slider.value() * step))

        # min label
        self.minLabel = QtWidgets.QLabel(parent)
        self.minLabel.setText(str(minimum))

        # max label
        self.maxLabel = QtWidgets.QLabel(parent)
        self.maxLabel.setText(str(maximum))

        # pairs
        self.pairs = pairs

        # vertical Layout
        self.vLayout = QtWidgets.QVBoxLayout()
        self.vLayout.addWidget(self.valueLabel, 0,
                               QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignBottom)
        self.vLayout.addWidget(self.slider, 0,
                               QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)

    def setup(self, horizontal_layout: QtWidgets.QHBoxLayout):
        horizontal_layout.addWidget(self.minLabel, 0,
                                    QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
        horizontal_layout.addLayout(self.vLayout, 0)
        horizontal_layout.addWidget(self.maxLabel, 0,
                                    QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)

    def spinBoxValueChanged(self):
        new_value = self.valueLabel.value()

        if new_value == self.currentValue or not EnhancedFloatQSlider.decimalMod(new_value,
                                                                                 self.step) or new_value % self.step == new_value:
            if self.slider.minimum() * self.step <= new_value <= self.slider.maximum() * self.step:
                self.slider.setValue(int(round(new_value, EnhancedFloatQSlider.getNumDecimals(new_value)) / self.step))
                self.currentValue = round(new_value, 2)
            else:
                self.valueLabel.setValue(round(self.currentValue, 2))
            return
        if self.slider.minimum() * self.step <= new_value <= self.slider.maximum() * self.step:

            self.slider.setValue(int(round((new_value / self.step), 0)))
            self.currentValue = round(new_value, 2)
        else:
            self.valueLabel.setValue(self.currentValue)

        if self.pairs is not None:
            self.pairUpdate()

    def setPairs(self, pairs: List['SliderGroupBox']):
        self.pairs = pairs

    def pairUpdate(self):
        new_value = self.valueLabel.value()
        for pair in self.pairs:
            if new_value == pair.slider.currentValue or not EnhancedFloatQSlider.decimalMod(new_value,
                                                                                            pair.slider.step) or new_value % pair.slider.step == new_value:
                if pair.slider.slider.minimum() * pair.slider.step <= new_value <= pair.slider.slider.maximum() * pair.slider.step:
                    pair.slider.slider.setValue(
                        int(round(new_value, EnhancedFloatQSlider.getNumDecimals(new_value)) / pair.slider.step))
                    pair.slider.currentValue = round(new_value, 2)
                else:
                    pair.slider.valueLabel.setValue(round(pair.slider.currentValue, 2))
                continue
            if pair.slider.slider.minimum() * pair.slider.step <= new_value <= pair.slider.slider.maximum() * pair.slider.step:
                pair.slider.slider.setValue(int(new_value / pair.slider.step))
                pair.slider.currentValue = round(new_value, 2)
            else:
                pair.slider.valueLabel.setValue(pair.slider.currentValue)

    @staticmethod
    def decimalMod(a: float, b: float) -> bool:
        if b > a:
            b, a = a, b
        a = round(a, 1)
        b = round(b, 1)
        while a != int(a) or b != int(b):
            a *= 10
            b *= 10
        a = int(a)
        b = int(b)
        try:
            return a % b == 0
        except ZeroDivisionError:
            return False

    @staticmethod
    def getNumDecimals(a: float) -> int:
        if a < 0:
            a = -a
        elif a == 0:
            return 0
        a -= int(a)
        decimals = 0
        while a < 1:
            a *= 10
            decimals += 1
        return decimals


class EnhancedIntQSlider:
    def __init__(self, parent: QtWidgets.QGroupBox, minimum: int, maximum: int, initial_value: int = None,
                 step: int = 1, prefix: str = "", suffix: str = "", minimum_width: int = 150,
                 pairs: List['SliderGroupBox'] = None):
        initial_value = initial_value or minimum

        # value label (QSpinBox)
        self.valueLabel = QtWidgets.QSpinBox(parent)
        self.valueLabel.setRange(minimum, maximum)
        self.valueLabel.setValue(initial_value)
        self.valueLabel.setSuffix(suffix)
        self.valueLabel.setPrefix(prefix)
        self.valueLabel.setSingleStep(step)
        self.valueLabel.setKeyboardTracking(False)
        self.valueLabel.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.valueLabel.setFixedWidth(130)
        self.valueLabel.setFixedHeight(20)
        self.currentValue = initial_value
        self.valueLabel.editingFinished.connect(lambda: self.spinBoxValueChanged())
        self.valueLabel.valueChanged.connect(lambda: self.spinBoxValueChanged())

        # slider
        self.slider = QtWidgets.QSlider(parent)
        self.slider.setRange(minimum, maximum)
        self.slider.setValue(initial_value)
        self.slider.setMinimumWidth(minimum_width)
        self.slider.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.slider.valueChanged.connect(lambda: self.valueLabel.setValue(self.slider.value()))

        # min label
        self.minLabel = QtWidgets.QLabel(parent)
        self.minLabel.setText(str(minimum))

        # max label
        self.maxLabel = QtWidgets.QLabel(parent)
        self.maxLabel.setText(str(maximum))

        # pairs
        self.pairs = pairs

        # vertical Layout
        self.vLayout = QtWidgets.QVBoxLayout()
        self.vLayout.addWidget(self.valueLabel, 0,
                               QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignBottom)
        self.vLayout.addWidget(self.slider, 0,
                               QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)

    def setup(self, horizontal_layout: QtWidgets.QHBoxLayout):
        horizontal_layout.addWidget(self.minLabel, 0,
                                    QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
        horizontal_layout.addLayout(self.vLayout, 0)
        horizontal_layout.addWidget(self.maxLabel, 0,
                                    QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)

    def spinBoxValueChanged(self):
        new_value = self.valueLabel.value()
        if new_value == self.currentValue:
            return
        elif self.slider.minimum() <= new_value <= self.slider.maximum():
            self.slider.setValue(new_value)
            self.currentValue = new_value
        else:
            self.valueLabel.setValue(self.currentValue)

        if self.pairs is not None:
            self.pairUpdate()

    def setPairs(self, pairs: List['SliderGroupBox']):
        self.pairs = pairs
        
    def pairUpdate(self):
        new_value = self.valueLabel.value()
        for pair in self.pairs:

            if new_value == pair.slider.currentValue:
                continue
            elif pair.slider.slider.minimum() <= new_value <= pair.slider.slider.maximum():
                print("updating")
                pair.slider.slider.setValue(new_value)
                pair.slider.currentValue = new_value
            else:
                pair.slider.valueLabel.setValue(pair.slider.currentValue)


class StatusGroupBox:
    def __init__(self, parent: QtWidgets.QGroupBox = None, name_text: str = "", info_text: str = ""):
        # group box
        self.groupbox = QtWidgets.QGroupBox(parent)
        self.groupbox.setProperty("class", "statusgb")
        self.groupbox.setMinimumSize(200, 50)
        self.groupbox.setMaximumSize(200, 50)

        # horizontal layout
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.groupbox)
        self.horizontalLayout.setContentsMargins(5, 5, 5, 5)

        # name label
        self.nameLabel = QtWidgets.QLabel(self.groupbox)
        self.nameLabel.setProperty("class", "nlabel")
        self.nameLabel.setText(name_text)
        self.nameLabel.setWordWrap(True)
        self.nameLabel.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

        # information label
        self.infoLabel = QtWidgets.QLabel(self.groupbox)
        self.infoLabel.setProperty("class", "ilabel")
        self.infoLabel.setText(info_text)
        self.infoLabel.setWordWrap(True)
        self.infoLabel.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

    def setup(self, row_horizontal_layout: QtWidgets.QHBoxLayout = None):
        self.horizontalLayout.addWidget(self.nameLabel, 0,
                                        QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.horizontalLayout.addWidget(self.infoLabel, 0,
                                        QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
        row_horizontal_layout.addWidget(self.groupbox, 0,
                                        QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
    def setValue(self, value):
        self.infoLabel.setText(str(value))


class CheckBoxGroupBox:
    def __init__(self, label: str, parent: QtWidgets.QGroupBox = None, min_size: Tuple[int, int] = (0, 0),
                 max_size: Tuple[int, int] = None, is_checked: bool = False,
                 expand_policy: Tuple[QtWidgets.QSizePolicy.Policy, QtWidgets.QSizePolicy.Policy] = (
                         QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Expanding),
                 v_expand: int = 0, h_expand: int = 0, pairs: List['CheckBoxGroupBox'] = None):
        size_policy = QtWidgets.QSizePolicy(*expand_policy)
        size_policy.setHorizontalStretch(h_expand)
        size_policy.setVerticalStretch(v_expand)

        # group box
        self.groupbox = QtWidgets.QGroupBox(parent)
        self.groupbox.setProperty("class", "parametergb")
        self.groupbox.setMinimumSize(*min_size)
        if max_size is not None:
            self.groupbox.setMaximumSize(*max_size)

        # horizontal layout
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.groupbox)
        self.horizontalLayout.setContentsMargins(5, 5, 5, 5)

        # label
        self.label = QtWidgets.QLabel(self.groupbox)
        self.label.setText(label)
        self.label.setWordWrap(True)
        self.label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label.setProperty("class", "parameterlabel")

        # check box
        self.checkBox = QtWidgets.QRadioButton(self.groupbox)
        self.checkBox.setChecked(is_checked)

        # pairs
        self.pairs = pairs
        self.checkBox.clicked.connect(lambda: self.user_toggle())

    def setup(self, row_horizontal_layout: QtWidgets.QHBoxLayout = None):
        self.horizontalLayout.addWidget(self.label, 1,
                                        QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.horizontalLayout.addWidget(self.checkBox, 1,
                                        QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
        row_horizontal_layout.addWidget(self.groupbox)

    def user_toggle(self):
        if self.pairs is not None:
            for pair in self.pairs:
                pair.checkBox.setChecked(self.checkBox.isChecked())

    def setPairs(self, pairs: List['CheckBoxGroupBox']):
        self.pairs = pairs


class EnhancedButton(QtWidgets.QPushButton):
    def __init__(self, checkable: bool = False, parent: QtWidgets.QWidget = None, labels: Tuple[str, str] = None,
                 label: str = None, fixed_size: Tuple[int, int] = None, layout: QtWidgets.QHBoxLayout = None,
                 property_class: str = "", pairs: List['EnhancedButton'] = None, callback_function:Callable = None, ui = None):
        super().__init__(parent)

        # self.callback_function = callback_function
        self.ui = ui
        self.setProperty("class", property_class)

        self.labels = labels
        self.setCheckable(checkable)

        if fixed_size is not None:
            self.setFixedSize(*fixed_size)

        if not checkable:
            self.setText(label)
        else:
            self.setText(labels[0])
            self.setChecked(False)

        if layout is not None:
            layout.addWidget(self, 0, QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)

        self.pairs = pairs

        # self.publisher = publisher

        self.clicked.connect(lambda: self.button_clicked(callback_function))

    def button_clicked(self, callback_function):
        callback_function()
        if self.isCheckable():
            pressed = self.isChecked()
            if pressed:
                self.setText(self.labels[1])
            else:
                self.setText(self.labels[0])

        if self.pairs is not None:
            self.pairs_click()


    def pairs_click(self):
        for pair in self.pairs:
            pair.button_clicked()


class SliderGroupBox:
    def __init__(self, property_class: str = "", parent: QtWidgets.QWidget = None,
                 margin: Tuple[int, int, int, int] = (10, 3, 10, 3),
                 label: str = "", min_value: float = 0, max_value: float = 100, step: float = 1, value: float = 0,
                 suffix: str = "", prefix: str = "", layout: QtWidgets.QHBoxLayout = None,
                 minimum_slider_width: int = 0,
                 minimum_height: int = 0, pairs: List['SliderGroupBox'] = None):

        # groupbox
        self.groupbox = QtWidgets.QGroupBox(parent)
        self.groupbox.setProperty("class", property_class)
        self.groupbox.setMinimumHeight(minimum_height)

        # horizontal layout
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.groupbox)
        self.horizontalLayout.setContentsMargins(*margin)

        # label
        self.label = QtWidgets.QLabel(self.groupbox)
        self.label.setText(label)
        self.label.setWordWrap(True)
        self.label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label.setProperty("class", "parameterlabel")

        self.setup(layout)

        # slider
        if step == 1:
            self.slider = EnhancedIntQSlider(parent=self.groupbox, minimum=int(min_value), maximum=int(max_value),
                                             initial_value=int(value), suffix=suffix, prefix=prefix,
                                             minimum_width=minimum_slider_width)
        else:
            self.slider = EnhancedFloatQSlider(parent=self.groupbox, minimum=min_value, maximum=max_value,
                                               initial_value=value, suffix=suffix, prefix=prefix, step=step,
                                               minimum_width=minimum_slider_width)
        self.slider.setup(self.horizontalLayout)

        # pairs
        self.pairs = pairs

    def setup(self, row_horizontal_layout: QtWidgets.QHBoxLayout = None):
        self.horizontalLayout.addWidget(self.label, 1,
                                        QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
        row_horizontal_layout.addWidget(self.groupbox)

    def setPairs(self, pairs: List['SliderGroupBox']):
        self.slider.setPairs(pairs)

#
# class CentralWidget(QtWidgets.QWidget):
#     def __init__(self, parent: QtWidgets.QWidget = None):
#         super().__init__(parent)
#         self.setFocusPolicy(QtCore.Qt.FocusPolicy.ClickFocus)


class ComboBoxGroupBox:
    def __init__(self, layout: QtWidgets.QHBoxLayout, label: str, choices: Iterable[str],
                 parent: QtWidgets.QWidget = None,
                 property_class: str = "", margin: Tuple[int, int, int, int] = (5, 5, 5, 5), default_index: int = 0,
                 pairs: List['ComboBoxGroupBox'] = None):
        # groupbox
        self.groupbox = QtWidgets.QGroupBox(parent)
        self.groupbox.setProperty("class", property_class)

        # horizontal layout
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.groupbox)
        self.horizontalLayout.setContentsMargins(*margin)

        # label
        self.label = QtWidgets.QLabel(self.groupbox)
        self.label.setText(label)
        self.label.setWordWrap(True)
        self.label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label.setProperty("class", "parameterlabel")

        # combo box
        self.comboBox = QtWidgets.QComboBox(self.groupbox)
        self.comboBox.addItems(choices)
        self.comboBox.setCurrentIndex(default_index)
        # self.comboBox.lineEdit().setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

        # pairs
        self.pairs = pairs
        self.comboBox.currentIndexChanged.connect(lambda: self.pairUpdate())

        self.horizontalLayout.addWidget(self.label, 1,
                                        QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.horizontalLayout.addWidget(self.comboBox, 1,
                                        QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(self.groupbox)

    def pairUpdate(self):
        if self.pairs is not None:
            for pair in self.pairs:
                pair.comboBox.setCurrentIndex(self.comboBox.currentIndex())

    def setPairs(self, pairs: List['ComboBoxGroupBox'] = None):
        self.pairs = pairs


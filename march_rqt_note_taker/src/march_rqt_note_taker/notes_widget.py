from python_qt_binding import loadUi, QtCore
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QFileDialog, QShortcut, QWidget
import rospy

from .entry import Entry


class NotesWidget(QWidget):

    def __init__(self, model, ui_file):
        super(NotesWidget, self).__init__()

        self._model = model
        self._can_save = True
        self._has_autosave = True

        loadUi(ui_file, self)

        self._model.rowsInserted.connect(lambda: [self.update_status(), self._set_saved(False)])
        self._model.rowsRemoved.connect(lambda: [self.update_status(), self._set_saved(False)])

        self.table_view.setModel(self._model)
        self.table_view.verticalScrollBar().rangeChanged.connect(self._handle_change_scroll)
        self._last_scroll_max = self.table_view.verticalScrollBar().maximum()

        self.input_field.returnPressed.connect(self._handle_insert_entry)

        self.take_button.clicked.connect(self._handle_start_take)

        self.load_button.clicked.connect(self._handle_load)
        self.save_button.clicked.connect(self._handle_save)
        self.autosave_check_box.stateChanged.connect(self._handle_autosave)
        self.autosave_check_box.setChecked(self._has_autosave)

        self._delete_shortcut = QShortcut(QKeySequence('Delete'), self)
        self._delete_shortcut.activated.connect(self._delete_selected)

        self._load_shortcut = QShortcut(QKeySequence('Ctrl+O'), self)
        self._load_shortcut.activated.connect(self._handle_load)

        self._save_shortcut = QShortcut(QKeySequence('Ctrl+S'), self)
        self._save_shortcut.activated.connect(self._handle_save)

    def _handle_insert_entry(self):
        entry = self.input_field.text().strip()
        if entry:
            self._model.insert_row(Entry(entry))
        self.input_field.clear()
        self.table_view.verticalScrollBar().setSliderPosition(self._last_scroll_max)

    def update_status(self):
        self.messages_label.setText('Displaying {0} messages'.format(self._model.rowCount()))

    def _handle_start_take(self):
        take = self.camera_spin_box.value()
        self._model.insert_row(Entry('Started camera take {0}'.format(take)))
        self.camera_spin_box.setValue(take + 1)
        self.table_view.verticalScrollBar().setSliderPosition(self._last_scroll_max)

    def _handle_change_scroll(self, scroll_min, scroll_max):
        if self.table_view.verticalScrollBar().value() == self._last_scroll_max:
            self.table_view.verticalScrollBar().setSliderPosition(scroll_max)
            self._last_scroll_max = scroll_max

    def _delete_selected(self):
        selection_model = self.table_view.selectionModel()
        if self.table_view.hasFocus() and selection_model.hasSelection():
            indices = [index for index in selection_model.selectedIndexes() if not index.column()]
            if indices and indices[0].isValid():
                self._model.remove_rows(indices[0].row(), len(indices))

    def _set_saved(self, saved):
        self._can_save = not saved
        self.save_button.setEnabled(not saved)

    def _handle_load(self):
        rospy.logwarn('Loading notes from a file is not yet implemented')

    def _handle_save(self):
        result = QFileDialog.getSaveFileName(self, 'Save File', '.', 'Minute files (*.txt)')
        file_name = result[0]
        if file_name:
            if file_name[-4:] != '.txt':
                file_name += '.txt'
            try:
                with open(file_name, 'w') as f:
                    f.write(str(self._model))
            except IOError as e:
                rospy.logwarn('Failed to open file: {0}'.format(e))
                return
            else:
                rospy.loginfo('Successfully written to file {0}'.format(file_name))
                self._set_saved(True)

    def _handle_autosave(self, state):
        self._has_autosave = state == QtCore.Qt.Checked

import sys
import csv
import random
from datetime import datetime
from typing import List, Dict, Any

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
    QHeaderView,
)


class TrialDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Waste Sorting Robot - 50 Trial Dashboard")
        self.resize(1600, 900)

        self.columns = [
            "Trial No",
            "Timestamp",
            "Waste Type",
            "Predicted Class",
            "Target Bin",
            "Correct",
            "Pick Time (s)",
            "Place Time (s)",
            "Total Cycle (s)",
            "Success",
            "Note",
        ]

        self.data: List[Dict[str, Any]] = []
        self.filtered_indexes: List[int] = []

        self._build_ui()
        self._load_empty_50_trials()
        self.refresh_table()
        self.update_kpis()

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QVBoxLayout()
        central.setLayout(main_layout)

        title = QLabel("Waste Sorting Robot Dashboard")
        title.setStyleSheet("font-size: 26px; font-weight: bold; padding: 8px;")
        main_layout.addWidget(title)

        subtitle = QLabel("50 trial test tracking for fake arm / detection / sorting performance")
        subtitle.setStyleSheet("font-size: 13px; color: #555; padding-left: 8px;")
        main_layout.addWidget(subtitle)

        main_layout.addWidget(self._build_kpi_section())
        main_layout.addWidget(self._build_controls_section())
        main_layout.addWidget(self._build_table_section())

    def _build_kpi_section(self) -> QWidget:
        box = QGroupBox("KPI Summary")
        layout = QGridLayout()
        box.setLayout(layout)

        self.total_trials_label = self._kpi_card("Total Trials", "0")
        self.success_rate_label = self._kpi_card("Success Rate", "0%")
        self.accuracy_label = self._kpi_card("Classification Accuracy", "0%")
        self.avg_cycle_label = self._kpi_card("Avg Cycle Time", "0.00 s")

        layout.addWidget(self.total_trials_label["box"], 0, 0)
        layout.addWidget(self.success_rate_label["box"], 0, 1)
        layout.addWidget(self.accuracy_label["box"], 0, 2)
        layout.addWidget(self.avg_cycle_label["box"], 0, 3)

        return box

    def _kpi_card(self, title: str, value: str) -> Dict[str, Any]:
        box = QGroupBox()
        layout = QVBoxLayout()
        box.setLayout(layout)
        box.setStyleSheet("""
            QGroupBox {
                border: 1px solid #cccccc;
                border-radius: 8px;
                margin-top: 5px;
                background: white;
            }
        """)

        title_label = QLabel(title)
        title_label.setStyleSheet("font-size: 13px; color: #666;")
        value_label = QLabel(value)
        value_label.setStyleSheet("font-size: 24px; font-weight: bold; color: #111;")

        layout.addWidget(title_label)
        layout.addWidget(value_label)

        return {"box": box, "value": value_label}

    def _build_controls_section(self) -> QWidget:
        box = QGroupBox("Controls")
        layout = QHBoxLayout()
        box.setLayout(layout)

        self.filter_waste = QComboBox()
        self.filter_waste.addItems(["All", "plastic", "paper", "metal"])

        self.filter_success = QComboBox()
        self.filter_success.addItems(["All", "success", "fail", "pending"])

        self.search_note = QLineEdit()
        self.search_note.setPlaceholderText("Search note...")

        self.apply_filter_btn = QPushButton("Apply Filter")
        self.apply_filter_btn.clicked.connect(self.apply_filters)

        self.clear_filter_btn = QPushButton("Clear Filter")
        self.clear_filter_btn.clicked.connect(self.clear_filters)

        self.generate_dummy_btn = QPushButton("Generate Dummy Data")
        self.generate_dummy_btn.clicked.connect(self.generate_dummy_data)

        self.save_csv_btn = QPushButton("Save CSV")
        self.save_csv_btn.clicked.connect(self.save_csv)

        self.load_csv_btn = QPushButton("Load CSV")
        self.load_csv_btn.clicked.connect(self.load_csv)

        self.reset_btn = QPushButton("Reset 50 Trials")
        self.reset_btn.clicked.connect(self.reset_trials)

        layout.addWidget(QLabel("Waste Type:"))
        layout.addWidget(self.filter_waste)
        layout.addWidget(QLabel("Success:"))
        layout.addWidget(self.filter_success)
        layout.addWidget(QLabel("Note:"))
        layout.addWidget(self.search_note)
        layout.addWidget(self.apply_filter_btn)
        layout.addWidget(self.clear_filter_btn)
        layout.addWidget(self.generate_dummy_btn)
        layout.addWidget(self.save_csv_btn)
        layout.addWidget(self.load_csv_btn)
        layout.addWidget(self.reset_btn)

        return box

    def _build_table_section(self) -> QWidget:
        box = QGroupBox("Trials")
        layout = QVBoxLayout()
        box.setLayout(layout)

        self.table = QTableWidget()
        self.table.setColumnCount(len(self.columns))
        self.table.setHorizontalHeaderLabels(self.columns)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setAlternatingRowColors(True)
        self.table.itemChanged.connect(self.on_item_changed)

        layout.addWidget(self.table)
        return box

    def _load_empty_50_trials(self):
        self.data.clear()
        for i in range(1, 51):
            self.data.append({
                "Trial No": i,
                "Timestamp": "",
                "Waste Type": "",
                "Predicted Class": "",
                "Target Bin": "",
                "Correct": "",
                "Pick Time (s)": "",
                "Place Time (s)": "",
                "Total Cycle (s)": "",
                "Success": "pending",
                "Note": "",
            })

    def reset_trials(self):
        reply = QMessageBox.question(
            self,
            "Reset",
            "All 50 trials will be reset. Continue?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self._load_empty_50_trials()
            self.refresh_table()
            self.update_kpis()

    def refresh_table(self):
        self.table.blockSignals(True)

        if not self.filtered_indexes:
            indexes = list(range(len(self.data)))
        else:
            indexes = self.filtered_indexes

        self.table.setRowCount(len(indexes))

        for row_idx, data_idx in enumerate(indexes):
            row_data = self.data[data_idx]
            for col_idx, col_name in enumerate(self.columns):
                value = str(row_data.get(col_name, ""))
                item = QTableWidgetItem(value)

                if col_name == "Trial No":
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)

                if col_name == "Success":
                    self._paint_success_cell(item, value)

                if col_name == "Correct":
                    self._paint_correct_cell(item, value)

                self.table.setItem(row_idx, col_idx, item)

        self.table.blockSignals(False)

    def _paint_success_cell(self, item: QTableWidgetItem, value: str):
        v = value.strip().lower()
        if v == "success":
            item.setBackground(QColor(198, 239, 206))
        elif v == "fail":
            item.setBackground(QColor(255, 199, 206))
        elif v == "pending":
            item.setBackground(QColor(255, 235, 156))

    def _paint_correct_cell(self, item: QTableWidgetItem, value: str):
        v = value.strip().lower()
        if v == "yes":
            item.setBackground(QColor(198, 239, 206))
        elif v == "no":
            item.setBackground(QColor(255, 199, 206))

    def on_item_changed(self, item: QTableWidgetItem):
        row = item.row()

        if not self.filtered_indexes:
            data_idx = row
        else:
            data_idx = self.filtered_indexes[row]

        col_name = self.columns[item.column()]
        self.data[data_idx][col_name] = item.text()

        self.auto_compute_total_cycle(data_idx)
        self.refresh_table()
        self.update_kpis()

    def auto_compute_total_cycle(self, idx: int):
        row = self.data[idx]
        try:
            pick = float(row["Pick Time (s)"]) if row["Pick Time (s)"] != "" else None
            place = float(row["Place Time (s)"]) if row["Place Time (s)"] != "" else None
            if pick is not None and place is not None:
                row["Total Cycle (s)"] = f"{pick + place:.2f}"
        except ValueError:
            pass

    def apply_filters(self):
        waste_filter = self.filter_waste.currentText().strip().lower()
        success_filter = self.filter_success.currentText().strip().lower()
        note_filter = self.search_note.text().strip().lower()

        self.filtered_indexes = []

        for i, row in enumerate(self.data):
            waste_ok = (waste_filter == "all" or row["Waste Type"].strip().lower() == waste_filter)
            success_ok = (success_filter == "all" or row["Success"].strip().lower() == success_filter)
            note_ok = (note_filter == "" or note_filter in row["Note"].strip().lower())

            if waste_ok and success_ok and note_ok:
                self.filtered_indexes.append(i)

        self.refresh_table()

    def clear_filters(self):
        self.filter_waste.setCurrentIndex(0)
        self.filter_success.setCurrentIndex(0)
        self.search_note.clear()
        self.filtered_indexes = []
        self.refresh_table()

    def update_kpis(self):
        completed = [r for r in self.data if str(r["Success"]).strip().lower() in ["success", "fail"]]
        successful = [r for r in self.data if str(r["Success"]).strip().lower() == "success"]
        classified = [r for r in self.data if str(r["Correct"]).strip().lower() in ["yes", "no"]]
        correct = [r for r in self.data if str(r["Correct"]).strip().lower() == "yes"]

        cycle_values = []
        for r in self.data:
            try:
                val = float(r["Total Cycle (s)"])
                cycle_values.append(val)
            except Exception:
                pass

        total_trials = len(self.data)
        success_rate = (len(successful) / len(completed) * 100) if completed else 0.0
        accuracy = (len(correct) / len(classified) * 100) if classified else 0.0
        avg_cycle = (sum(cycle_values) / len(cycle_values)) if cycle_values else 0.0

        self.total_trials_label["value"].setText(str(total_trials))
        self.success_rate_label["value"].setText(f"{success_rate:.1f}%")
        self.accuracy_label["value"].setText(f"{accuracy:.1f}%")
        self.avg_cycle_label["value"].setText(f"{avg_cycle:.2f} s")

    def generate_dummy_data(self):
        waste_types = ["plastic", "paper", "metal"]
        target_bins = {"plastic": "red", "paper": "green", "metal": "yellow"}

        for i in range(len(self.data)):
            waste = random.choice(waste_types)

            pred_ok = random.random() < 0.86
            pred = waste if pred_ok else random.choice([w for w in waste_types if w != waste])

            correct = "yes" if pred == waste else "no"
            success = "success" if random.random() < 0.88 else "fail"

            pick_t = round(random.uniform(1.1, 3.0), 2)
            place_t = round(random.uniform(1.0, 2.8), 2)

            self.data[i] = {
                "Trial No": i + 1,
                "Timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "Waste Type": waste,
                "Predicted Class": pred,
                "Target Bin": target_bins[waste],
                "Correct": correct,
                "Pick Time (s)": f"{pick_t:.2f}",
                "Place Time (s)": f"{place_t:.2f}",
                "Total Cycle (s)": f"{pick_t + place_t:.2f}",
                "Success": success,
                "Note": random.choice([
                    "normal run",
                    "minor delay",
                    "bin approach ok",
                    "detection stable",
                    "pick smooth",
                    ""
                ]),
            }

        self.filtered_indexes = []
        self.refresh_table()
        self.update_kpis()

    def save_csv(self):
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save CSV",
            "trial_dashboard.csv",
            "CSV Files (*.csv)"
        )
        if not file_path:
            return

        try:
            with open(file_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=self.columns)
                writer.writeheader()
                for row in self.data:
                    writer.writerow(row)

            QMessageBox.information(self, "Success", "CSV saved successfully.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save CSV:\n{e}")

    def load_csv(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Load CSV",
            "",
            "CSV Files (*.csv)"
        )
        if not file_path:
            return

        try:
            loaded_data = []
            with open(file_path, "r", newline="", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    normalized = {}
                    for col in self.columns:
                        normalized[col] = row.get(col, "")
                    loaded_data.append(normalized)

            if not loaded_data:
                raise ValueError("CSV is empty.")

            self.data = loaded_data

            if len(self.data) < 50:
                start = len(self.data) + 1
                for i in range(start, 51):
                    self.data.append({
                        "Trial No": i,
                        "Timestamp": "",
                        "Waste Type": "",
                        "Predicted Class": "",
                        "Target Bin": "",
                        "Correct": "",
                        "Pick Time (s)": "",
                        "Place Time (s)": "",
                        "Total Cycle (s)": "",
                        "Success": "pending",
                        "Note": "",
                    })

            self.filtered_indexes = []
            self.refresh_table()
            self.update_kpis()

            QMessageBox.information(self, "Success", "CSV loaded successfully.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load CSV:\n{e}")


def main():
    app = QApplication(sys.argv)
    window = TrialDashboard()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

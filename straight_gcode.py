import tkinter as tk
from tkinter import ttk, filedialog, messagebox

class DrawingApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Custom Drawing to G-code Converter")
        self.root.geometry("900x700")

        # Variables
        self.drawing_mode = "line"  # Default drawing mode: line, diagonal, free
        self.lines = []  # List to store drawn lines
        self.current_line = None  # For temporary line during drawing
        self.drawing = False
        self.start_x = None
        self.start_y = None
        self.delete_mode = False  # Flag for delete mode
        self.red_dots = []  # List to store red dot objects

        # Create UI
        self.create_widgets()

    def create_widgets(self):
        # Main frame
        main_frame = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Control panel
        control_frame = ttk.Frame(main_frame)
        main_frame.add(control_frame, weight=30)

        # Display panel
        display_frame = ttk.Frame(main_frame)
        main_frame.add(display_frame, weight=70)

        # --- CONTROL PANEL ---
        # Drawing tools
        tools_frame = ttk.LabelFrame(control_frame, text="Drawing Tools", padding=10)
        tools_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Button(tools_frame, text="Line", command=lambda: self.set_drawing_mode("line")).pack(side=tk.LEFT, padx=5)
        ttk.Button(tools_frame, text="Diagonal", command=lambda: self.set_drawing_mode("diagonal")).pack(side=tk.LEFT, padx=5)
        ttk.Button(tools_frame, text="Free Draw", command=lambda: self.set_drawing_mode("free")).pack(side=tk.LEFT, padx=5)
        ttk.Button(tools_frame, text="Xóa", command=self.toggle_delete_mode).pack(side=tk.LEFT, padx=5)

        # Workspace limits
        workspace_frame = ttk.LabelFrame(control_frame, text="Workspace Limits (cm)", padding=10)
        workspace_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(workspace_frame, text="X-axis:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        self.x_min_var = tk.DoubleVar(value=2)
        self.x_max_var = tk.DoubleVar(value=18)
        ttk.Label(workspace_frame, text="From:").grid(row=0, column=1, sticky="e")
        ttk.Entry(workspace_frame, textvariable=self.x_min_var, width=6).grid(row=0, column=2, padx=2)
        ttk.Label(workspace_frame, text="to:").grid(row=0, column=3, sticky="e")
        ttk.Entry(workspace_frame, textvariable=self.x_max_var, width=6).grid(row=0, column=4, padx=2)

        ttk.Label(workspace_frame, text="Y-axis:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        self.y_min_var = tk.DoubleVar(value=15)
        self.y_max_var = tk.DoubleVar(value=30)
        ttk.Label(workspace_frame, text="From:").grid(row=1, column=1, sticky="e")
        ttk.Entry(workspace_frame, textvariable=self.y_min_var, width=6).grid(row=1, column=2, padx=2)
        ttk.Label(workspace_frame, text="to:").grid(row=1, column=3, sticky="e")
        ttk.Entry(workspace_frame, textvariable=self.y_max_var, width=6).grid(row=1, column=4, padx=2)

        # Speed control
        speed_frame = ttk.LabelFrame(control_frame, text="Drawing Speed", padding=10)
        speed_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(speed_frame, text="Speed:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        self.speed_var = tk.IntVar(value=500)
        ttk.Scale(speed_frame, from_=100, to=2000, variable=self.speed_var, orient="horizontal").grid(row=0, column=1, sticky="we")

        # Generate G-code button
        ttk.Button(control_frame, text="Generate G-code", command=self.generate_gcode).pack(fill=tk.X, padx=5, pady=10)

        # --- DISPLAY PANEL ---
        # Canvas for drawing
        self.canvas = tk.Canvas(display_frame, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Bind mouse events
        self.canvas.bind("<ButtonPress-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<Motion>", self.on_motion)

    def set_drawing_mode(self, mode):
        self.drawing_mode = mode
        self.delete_mode = False

    def toggle_delete_mode(self):
        self.delete_mode = not self.delete_mode
        if self.delete_mode:
            self.canvas.config(cursor="cross")
        else:
            self.canvas.config(cursor="arrow")

    def on_press(self, event):
        if self.delete_mode:
            self.delete_line(event)
            return

        self.drawing = True
        self.start_x = self.canvas.canvasx(event.x)
        self.start_y = self.canvas.canvasy(event.y)
        if self.drawing_mode == "free":
            self.current_line = [self.start_x, self.start_y]
            self.lines.append(self.current_line)
        else:
            self.current_line = self.canvas.create_line(self.start_x, self.start_y, self.start_x, self.start_y, fill="blue", width=2, tags="temp_line")
        self.update_red_dots()  # Cập nhật điểm đỏ ngay khi bắt đầu vẽ

    def on_drag(self, event):
        if not self.drawing or self.delete_mode:
            return
        cur_x = self.canvas.canvasx(event.x)
        cur_y = self.canvas.canvasy(event.y)
        if self.drawing_mode == "line":
            dx = abs(cur_x - self.start_x)
            dy = abs(cur_y - self.start_y)
            if dx > dy:
                self.canvas.coords(self.current_line, self.start_x, self.start_y, cur_x, self.start_y)
            else:
                self.canvas.coords(self.current_line, self.start_x, self.start_y, self.start_x, cur_y)
        elif self.drawing_mode == "diagonal":
            self.canvas.coords(self.current_line, self.start_x, self.start_y, cur_x, cur_y)
        elif self.drawing_mode == "free":
            self.current_line.extend([cur_x, cur_y])
            self.canvas.create_line(self.current_line[-4:], fill="black", width=2)
        self.update_red_dots()  # Cập nhật điểm đỏ trong lúc kéo chuột

    def on_release(self, event):
        if self.delete_mode:
            return
        self.drawing = False
        end_x = self.canvas.canvasx(event.x)
        end_y = self.canvas.canvasy(event.y)
        if self.drawing_mode == "line":
            dx = abs(end_x - self.start_x)
            dy = abs(end_y - self.start_y)
            if dx > dy:
                final_end_x = end_x
                final_end_y = self.start_y
            else:
                final_end_x = self.start_x
                final_end_y = end_y
            self.canvas.coords(self.current_line, self.start_x, self.start_y, final_end_x, final_end_y)
            self.lines.append((self.start_x, self.start_y, final_end_x, final_end_y))
            self.canvas.itemconfig(self.current_line, fill="black", tags="")
        elif self.drawing_mode == "diagonal":
            self.canvas.coords(self.current_line, self.start_x, self.start_y, end_x, end_y)
            self.lines.append((self.start_x, self.start_y, end_x, end_y))
            self.canvas.itemconfig(self.current_line, fill="black", tags="")
        elif self.drawing_mode == "free":
            pass
        self.update_red_dots()  # Cập nhật điểm đỏ sau khi thả chuột

    def on_motion(self, event):
        if self.delete_mode or self.drawing:
            return
        cur_x = self.canvas.canvasx(event.x)
        cur_y = self.canvas.canvasy(event.y)
        unconnected_points = self.find_unconnected_points()
        close_points = [p for p in unconnected_points if ((p[0] - cur_x)**2 + (p[1] - cur_y)**2)**0.5 < 10]
        self.update_red_dots(close_points)

    def find_unconnected_points(self):
        all_endpoints = []
        for line in self.lines:
            if len(line) == 4:  # Line hoặc diagonal
                start = (round(line[0], 2), round(line[1], 2))
                end = (round(line[2], 2), round(line[3], 2))
                all_endpoints.append(start)
                all_endpoints.append(end)
        point_counts = {}
        for point in all_endpoints:
            point_counts[point] = point_counts.get(point, 0) + 1
        unconnected_points = [point for point, count in point_counts.items() if count == 1]
        return unconnected_points

    def update_red_dots(self, points=None):
        # Xóa các điểm đỏ hiện có
        for dot in self.red_dots:
            self.canvas.delete(dot)
        self.red_dots = []
        if points is None:
            points = self.find_unconnected_points()
        # Vẽ các điểm đỏ cho các điểm chưa nối
        for p in points:
            x, y = p
            dot = self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill="red")
            self.red_dots.append(dot)

    def delete_line(self, event):
        x = self.canvas.canvasx(event.x)
        y = self.canvas.canvasy(event.y)
        items = self.canvas.find_overlapping(x-1, y-1, x+1, y+1)
        for item in items:
            if "temp_line" in self.canvas.gettags(item):
                continue
            self.canvas.delete(item)
            # Xóa khỏi danh sách lines
            for i, line in enumerate(self.lines):
                if len(line) == 4 and self.canvas.coords(item) == list(line):
                    self.lines.pop(i)
                    break
        self.update_red_dots()  # Cập nhật điểm đỏ sau khi xóa

    def generate_gcode(self):
        if not self.lines:
            messagebox.showwarning("Warning", "No lines to generate G-code!")
            return

        # Lấy kích thước không gian làm việc
        x_min = self.x_min_var.get()
        x_max = self.x_max_var.get()
        y_min = self.y_min_var.get()
        y_max = self.y_max_var.get()
        workspace_width = x_max - x_min
        workspace_height = y_max - y_min

        # Lấy kích thước canvas
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()

        # Tính tỷ lệ
        scale_x = workspace_width / canvas_width
        scale_y = workspace_height / canvas_height

        # Tạo G-code
        gcode = []
        gcode.append("M5")  # Nâng bút

        for line in self.lines:
            if len(line) == 4:  # Line hoặc diagonal
                start_x, start_y, end_x, end_y = line
                real_start_x = x_min + start_x * scale_x
                real_start_y = y_min + (canvas_height - start_y) * scale_y
                real_end_x = x_min + end_x * scale_x
                real_end_y = y_min + (canvas_height - end_y) * scale_y
                gcode.append(f"G0 X{real_start_x:.3f} Y{real_start_y:.3f}")
                gcode.append("M3")  # Hạ bút
                gcode.append(f"G1 X{real_end_x:.3f} Y{real_end_y:.3f}")
                gcode.append("M5")  # Nâng bút
            elif len(line) > 4:  # Free draw
                for i in range(0, len(line) - 2, 2):
                    x1, y1 = line[i], line[i+1]
                    x2, y2 = line[i+2], line[i+3]
                    real_x1 = x_min + x1 * scale_x
                    real_y1 = y_min + (canvas_height - y1) * scale_y
                    real_x2 = x_min + x2 * scale_x
                    real_y2 = y_min + (canvas_height - y2) * scale_y
                    gcode.append(f"G0 X{real_x1:.3f} Y{real_y1:.3f}")
                    gcode.append("M3")  # Hạ bút
                    gcode.append(f"G1 X{real_x2:.3f} Y{real_y2:.3f}")
                    gcode.append("M5")  # Nâng bút

        # Lưu G-code
        file_path = filedialog.asksaveasfilename(
            title="Save G-code",
            defaultextension=".gcode",
            filetypes=[("G-code files", "*.gcode"), ("All files", "*.*")]
        )

        if file_path:
            with open(file_path, 'w') as f:
                f.write('\n'.join(gcode))
            messagebox.showinfo("Success", f"G-code saved to {file_path}")

if __name__ == "__main__":
    root = tk.Tk()
    app = DrawingApp(root)
    root.mainloop()
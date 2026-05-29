from vex import *

class XYPlotter:
    def __init__(self, min_x=None, max_x=None, min_y=None, max_y=None, square_aspect=False, invert_y=False, invert_x=False):
        '''
        X and Y follow standard screen coordinates: (0,0) is top-left, X increases right, Y increases down.
        '''
        # Screen dimensions and margins
        self.screen_width = 480
        self.screen_height = 240
        self.margin_left = 5
        self.margin_right = 5
        self.margin_top = 5
        self.margin_bottom = 5
        self.actual_width = self.screen_width - self.margin_left - self.margin_right
        self.actual_height = self.screen_height - self.margin_top - self.margin_bottom
        self.aspect_ratio = self.actual_width / self.actual_height

        # Data series
        self.s1x, self.s1y = [], []
        self.s2x, self.s2y = [], []
        self.s3x, self.s3y = [], []

        # Data limits
        limit_set_count = 0
        if min_x is not None: limit_set_count += 1
        if max_x is not None: limit_set_count += 1
        if min_y is not None: limit_set_count += 1
        if max_y is not None: limit_set_count += 1
        if (limit_set_count > 0) and (limit_set_count < 4):
            raise ValueError("Must set all limits or none for auto-scaling")

        self.auto_scale = True if limit_set_count == 0 else False
        self.invert_y = invert_y
        self.invert_x = invert_x
        self.square_aspect = square_aspect
        self.rotate = 0 # TODO: Set angle to rotate the plot (0, 90, 180, 270)

        self.min_x = 1 if min_x is None else min_x
        self.max_x = -1 if max_x is None else max_x
        self.min_y = 1 if min_y is None else min_y
        self.max_y = -1 if max_y is None else max_y

    # -----------------------------
    # Data input
    # -----------------------------
    def add_data_point_series1(self, x, y):
        self.s1x.append(x)
        self.s1y.append(y)
        self.update_limits(x, y)

    def add_data_point_series2(self, x, y):
        self.s2x.append(x)
        self.s2y.append(y)
        self.update_limits(x, y)

    def add_data_point_series3(self, x, y):
        self.s3x.append(x)
        self.s3y.append(y)
        self.update_limits(x, y)

    def clear_data(self, series=0):
        if series == 1 or series == 0:
            self.s1x.clear(); self.s1y.clear()
        if series == 2 or series == 0:
            self.s2x.clear(); self.s2y.clear()
        if series == 3 or series == 0:
            self.s3x.clear(); self.s3y.clear()

        if series == 0:
            self.reset_limits()

    def update_limits(self, x, y):
        if not self.auto_scale:
            return
        self.min_x = min(x, self.min_x)
        self.max_x = max(x, self.max_x)
        self.min_y = min(y, self.min_y)
        self.max_y = max(y, self.max_y)

    def reset_limits(self):
        if not self.auto_scale:
            return
        self.min_x = 1
        self.max_x = -1
        self.min_y = 1
        self.max_y = -1   

    # -----------------------------
    # Plotting
    # -----------------------------
    def draw_plot(self, screen: Brain.Lcd):
        """
        screen: an object with methods:
            clear(), draw_rect(x,y,w,h,color),
            draw_circle(x,y,r,color),
            draw_line(x1,y1,x2,y2,color)
        """
        self.update_scaling()

        screen.clear_screen()

        plot_x = self.margin_left
        plot_y = self.margin_top
        plot_width = self.actual_width
        plot_height = self.actual_height

        # Draw border
        screen.draw_rectangle(plot_x, plot_y, plot_width, plot_height, Color.BLACK)

        # Draw each series
        self.draw_series(screen, self.s1x, self.s1y, Color.RED)
        self.draw_series(screen, self.s2x, self.s2y, Color.BLUE)
        self.draw_series(screen, self.s3x, self.s3y, Color.GREEN)

    def draw_overlay(self, screen: Brain.Lcd, x, y, type="circle", size=1, color=Color.YELLOW):
        # Draw a circle over a data point (e.g. as a confidence interval)
        screen_x = self.data_to_screen_x(x)
        screen_y = self.data_to_screen_y(y)
        screen_size = int(size * (self.actual_width / self.data_x_range)) # Scale size based on data range and screen width
        if type == "circle":
            screen.draw_circle(screen_x, screen_y, screen_size, color)
        elif type == "square":
            screen.draw_rectangle(screen_x - screen_size, screen_y - screen_size, screen_size * 2, screen_size * 2, color)

    # -----------------------------
    # Scaling and limits
    # -----------------------------
    def update_scaling(self):
        
        if self.min_x > self.max_x or self.min_y > self.max_y:
            self.min_x, self.max_x = 0, 100
            self.min_y, self.max_y = 0, 100

        # Avoid zero ranges
        if self.min_x == self.max_x:
            self.max_x += 1
        if self.min_y == self.max_y:
            self.max_y += 1

        # Data centering and normalization
        # - Data will be normalized to range of -0.5 to 0.5 before being expanded to screen dimensions
        # - 0.0 will be at the center of the appropriate axis
        # - For square aspect ratio the smaller range will be padded to center the plot and the same range scaling is applied to both axes
        self.data_x_center = 0.0
        self.data_y_center = 0.0
        self.data_x_range = self.max_x - self.min_x
        self.data_y_range = self.max_y - self.min_y
        if self.auto_scale and self.square_aspect:
            data_range = max(self.data_x_range, self.data_y_range)
            if self.data_x_range < data_range:
                self.data_x_center = (data_range - self.data_x_range) / 2
                self.data_x_range = data_range
            elif self.data_y_range < data_range:
                self.data_y_center = (data_range - self.data_y_range) / 2
                self.data_y_range = data_range

        # print(self.min_x, self.max_x, self.min_y, self.max_y)

    def data_to_screen_x(self, x):
        plot_width = self.actual_width
        plot_x_center = self.margin_left + plot_width / 2
        expansion = plot_width
        if self.auto_scale and self.square_aspect:
            expansion = min(self.actual_height, self.actual_height) # Use smaller dimension for square aspect ratio

        normalized = -0.5 + (x + self.data_x_center - self.min_x) / self.data_x_range
        if self.invert_x:
            normalized = -normalized

        return int(plot_x_center + normalized * expansion)

    def data_to_screen_y(self, y):
        plot_height = self.actual_height
        plot_y_center = self.margin_top + plot_height / 2
        expansion = plot_height
        if self.auto_scale and self.square_aspect:
            expansion = min(self.actual_height, self.actual_height) # Use smaller dimension for square aspect ratio

        normalized = -0.5 + (y + self.data_y_center - self.min_y) / self.data_y_range
        if self.invert_y:
            normalized = -normalized

        return int(plot_y_center + normalized * expansion)

    # -----------------------------
    # Draw a series
    # -----------------------------
    def draw_series(self, screen: Brain.Lcd, xs, ys, color):
        if not xs:
            return
        
        screen.set_pen_color(color)

        prev_x = None
        prev_y = None

        for x, y in zip(xs, ys):
            sx = self.data_to_screen_x(x)
            sy = self.data_to_screen_y(y)

            screen.draw_circle(sx, sy, 3, color)

            if prev_x is not None and prev_y is not None:
                screen.draw_line(prev_x, prev_y, sx, sy)

            prev_x, prev_y = sx, sy

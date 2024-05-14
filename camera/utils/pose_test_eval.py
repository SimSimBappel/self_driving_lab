import matplotlib.pyplot as plt
import numpy as np
from matplotlib.image import imread
from tkinter import Tk, filedialog
import os

def select_image():
    """ Opens a file dialog for image selection. """
    root = Tk()
    root.withdraw()  # Hide the main window.
    file_path = filedialog.askopenfilename()
    return file_path

def main():
    coords = []
    file_path = select_image()
    if not file_path:
        print("No image selected.")
        return

    img = imread(file_path)
    fig, ax = plt.subplots(figsize=(20, 11))  # Larger figure size
    ax.imshow(img)
    plt.title("Select one corner of the known-size square (0.035m)")

    def onclick(event):
        """ Event handler for mouse clicks on the plot. """
        if event.xdata is not None and event.ydata is not None:
            coords.append((event.xdata, event.ydata))
            # plt.plot(event.xdata, event.ydata, 'ro')
            num_points = len(coords)
            if num_points == 1:
                plt.plot(event.xdata, event.ydata, 'bo')
                plt.title("Select the opposite corner of the calibration square")
            elif num_points == 2:
                plt.plot(event.xdata, event.ydata, 'bo')
                plt.title("Select the first point (reference point)")
            elif num_points == 3:
                plt.plot(event.xdata, event.ydata, 'rx')
                plt.title("Select several other points (click middle button to finish)")
            elif num_points >= 13:
                plt.plot(event.xdata, event.ydata, 'ro')
                plt.title("Maximum number of points reached.")
            else:
                plt.plot(event.xdata, event.ydata, 'ro')
                plt.title("Select several other points (click middle button to finish)")
            plt.draw()

    def finish_selection(event):
        """ Event handler to finish point selection. """
        if event.button == 2:  # Middle mouse button
            if len(coords) > 2:
                # Calculate pixels-per-meter using the known size square
                square_length_pixels = np.linalg.norm(np.array(coords[0]) - np.array(coords[1]))
                known_size_meters = 0.035  # known size of the square in meters
                pixels_per_meter = square_length_pixels / known_size_meters
                
                calculate_metrics(coords, pixels_per_meter, fig, file_path)
            else:
                print("Insufficient points to calculate metrics.")
            plt.close()

    def calculate_metrics(coords, pixels_per_meter, fig, image_path):
        """ Calculate bias and mean error relative to the first point in meters. """
        first_point = np.array(coords[2])  # Third point is the first reference point
        subsequent_points = np.array(coords[3:])
        
        # Calculate distances from the first point to each of the subsequent points
        distances = [np.linalg.norm(point - first_point) / pixels_per_meter for point in subsequent_points]

        mean_distance = np.mean(distances)
        # Compute the mean position of subsequent points
        mean_of_points = np.mean(subsequent_points, axis=0)
        
        # Calculate bias in x and y separately
        bias_x = (mean_of_points[0] - first_point[0]) / pixels_per_meter
        bias_y = (mean_of_points[1] - first_point[1]) / pixels_per_meter

        print("Mean Distance Error (meters): ", mean_distance)
        print("Bias from first point to mean of other points (meters):")
        print("  Bias X (meters):", bias_x)
        print("  Bias Y (meters):", bias_y)
        
        # Define the distance threshold (in meters)
        distance_threshold = 0.011
        within_threshold_count = sum(1 for d in distances if d <= distance_threshold)
        
        print(f"Number of points within {distance_threshold} meters of the reference point: {within_threshold_count}")

        title = (f"Mean Distance Error: {mean_distance*1000:.1f} mm\n"
                 f"Bias X: {bias_x*1000:.1f} mm, Bias Y: {bias_y*1000:.1f} mm\n"
                 f"Points within {distance_threshold*1000} mm: {within_threshold_count}/{len(distances)}\n")
        ax.set_title(title)
        # Save the annotated image before closing the figure
        save_annotated_image(fig, image_path)

    def save_annotated_image(fig, image_path):
        """ Saves the figure with annotations to a file. """
        base, ext = os.path.splitext(image_path)
        output_path = f"{base}_dots{ext}"
        fig.savefig(output_path, dpi=300)  # Using a higher DPI for better file quality
        print(f"Annotated image saved as: {output_path}")

    def onscroll(event):
        """ Event handler for scrolling to enable zooming. """
        ax = plt.gca()
        if event.button == 'up':
            xlim = ax.get_xlim()
            ylim = ax.get_ylim()
            new_xlim = (xlim[0] * 0.9, xlim[1] * 0.9)
            new_ylim = (ylim[0] * 0.9, ylim[1] * 0.9)
            ax.set_xlim(new_xlim)
            ax.set_ylim(new_ylim)
        elif event.button == 'down':
            xlim = ax.get_xlim()
            ylim = ax.get_ylim()
            new_xlim = (xlim[0] * 1.1, xlim[1] * 1.1)
            new_ylim = (ylim[0] * 1.1, ylim[1] * 1.1)
            ax.set_xlim(new_xlim)
            ax.set_ylim(new_ylim)
        plt.draw()

    fig.canvas.mpl_connect('button_press_event', onclick)
    fig.canvas.mpl_connect('button_release_event', finish_selection)
    fig.canvas.mpl_connect('scroll_event', onscroll)

    plt.show()

if __name__ == '__main__':
    main()

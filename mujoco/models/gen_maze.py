#!/usr/bin/env python3

import sys
import argparse
from enum import StrEnum
from pathlib import Path

CELL_CHAR_WIDTH = 4
CELL_CHAR_HEIGHT = 2

class MazeSymbol(StrEnum):
    POST = 'o'
    WALL_HORIZONTAL = '---'
    WALL_HORIZONTAL_EMPTY = '   '
    WALL_VERTICAL = '|'
    WALL_VERTICAL_EMPTY = ''
    START = 'S'
    GOAL = 'G'

class MazeParser:
    def __init__(self, cell_size: float = 0.18, wall_thickness: float = 0.012, wall_height: float = 0.05):
        self.cell_size = cell_size
        self.wall_thickness = wall_thickness
        self.wall_height = wall_height
        self.width = 0
        self.height = 0
        self.start = None
        self.goals = []

    def parse_maze_file(self, maze_file: str) -> list[str]:
        with open(maze_file, 'r') as f:
            lines = [line.rstrip() for line in f.readlines()]

        if not lines:
            raise ValueError("Empty maze file")

        self.maze_width = (len(lines[0]) - 1) // CELL_CHAR_WIDTH
        self.maze_height = (len(lines) - 1) // CELL_CHAR_HEIGHT

        print(f"Detected maze dimensions: {self.maze_width}x{self.maze_height}")
        return lines

    def extract_horizontal_walls(self, lines: list[str]) -> list[list[bool]]:
        horizontal_walls = []
        for i in range(0, len(lines), 2):
            row = []
            for j in range(0, len(lines[i]), 4):
                if lines[i][j + 1:j + 4] == MazeSymbol.WALL_HORIZONTAL:
                    row.append(True)
                else:
                    row.append(False)
            horizontal_walls.append(row)
        return horizontal_walls


    def extract_vertical_walls(self, lines: list[str]) -> list[list[bool]]:
        vertical_walls = []
        for i in range(1, len(lines), 2):
            row = []
            for j in range(0, len(lines[i]), 4):
                if lines[i][j] == MazeSymbol.WALL_VERTICAL:
                    row.append(True)
                else:
                    row.append(False)
            vertical_walls.append(row)
        return vertical_walls

    def find_start_and_goals(self, lines: list[str]) -> tuple[tuple[int, int], list[tuple[int, int]]]:
        start = None
        goals = []

        row = 0
        col = 0

        inverted_lines = lines[::-1]

        for i in range(1, len(inverted_lines), 2):
            for j in range(2, len(inverted_lines[i]), 4):
                if inverted_lines[i][j] == MazeSymbol.START:
                    if start is not None:
                        raise ValueError("Multiple start points found")
                    start = (row, col)
                elif inverted_lines[i][j] == MazeSymbol.GOAL:
                    goals.append((row, col))
                col += 1
            row += 1
            col = 0

        self.start = start
        self.goals = goals

        return start, goals

    def is_start_or_goal(self, row: int, col: int) -> bool:
        if (row, col) == self.start:
            return True
        if (row, col) in self.goals:
            return True
        return False

    def is_post_on_start_cell(self, row: int, col: int) -> bool:
        if self.is_start_or_goal(row, col) or \
           self.is_start_or_goal(row - 1, col) or \
           self.is_start_or_goal(row, col - 1) or \
           self.is_start_or_goal(row - 1, col - 1):
            return True

        return False

    def is_horizontal_wall_on_start_cell(self, row: int, col: int) -> bool:
        if self.is_start_or_goal(row  - 1, col) or \
           self.is_start_or_goal(row, col):
            return True

        return False

    def is_vertical_wall_on_start_cell(self, row: int, col: int) -> bool:
        if self.is_start_or_goal(row, col) or \
           self.is_start_or_goal(row, col - 1):
            return True

        return False

    def generate_posts_xml(self) -> list[str]:
        xml = []

        for row in range(self.maze_height + 1):
            for col in range(self.maze_width + 1):
                x = col * self.cell_size
                y = row * self.cell_size

                class_name = 'post_w' if self.is_post_on_start_cell(row, col) else 'post'
                xml.append(
                    f'    <geom class="{class_name}" name="post_{row}_{col}" '
                    f'pos="{x:.6f} {y:.6f} {self.wall_height / 2:.6f}"/>'
                )
        return xml

    def generate_horizontal_walls_xml(self, horizontal_walls: list[list[bool]]) -> list[str]:
        xml = []
        inverted_horizontal_walls = horizontal_walls[::-1]

        for row in range(len(inverted_horizontal_walls)):
            for col in range(len(inverted_horizontal_walls[row])):
                if inverted_horizontal_walls[row][col]:
                    x = col * self.cell_size + self.cell_size / 2
                    y = row * self.cell_size

                    class_name = 'hwall_w' if self.is_horizontal_wall_on_start_cell(row, col) else 'hwall'
                    xml.append(
                        f'    <geom class="{class_name}" name="hwall_{row}_{col}" '
                        f'pos="{x:.6f} {y:.6f} {self.wall_height / 2:.6f}"/>'
                    )
        return xml

    def generate_vertical_walls_xml(self, vertical_walls: list[list[bool]]) -> list[str]:
        xml = []
        inverted_vertical_walls = vertical_walls[::-1]

        for row in range(len(inverted_vertical_walls)):
            for col in range(len(inverted_vertical_walls[row])):
                if inverted_vertical_walls[row][col]:
                    x = col * self.cell_size
                    y = row * self.cell_size + self.cell_size / 2

                    class_name = 'vwall_w' if self.is_vertical_wall_on_start_cell(row, col) else 'vwall'
                    xml.append(
                        f'    <geom class="{class_name}" name="vwall_{row}_{col}" '
                        f'pos="{x:.6f} {y:.6f} {self.wall_height / 2:.6f}"/>'
                    )
        return xml

    def add_header(self) -> str:
        xml = []
        xml.append('<!-- Generated maze for MuJoCo -->')
        xml.append(f'<!-- Maze dimensions: {self.maze_width}x{self.maze_height} -->')
        xml.append(f'<!-- Cell size: {self.cell_size:.2f} m, Wall thickness: {self.wall_thickness:.4f} m, Wall height: {self.wall_height:.2f} m -->\n')
        return '\n'.join(xml)

    def generate_assets(self) -> str:
        xml = []
        xml.append('  <asset>')
        xml.append('    <texture name="walls" type="cube" builtin="flat" width="512" height="512" rgb1="1 1 1" rgb2="1 0 0"/>')
        xml.append('    <material name="walls" texture="walls"/>')
        xml.append('  </asset>\n')
        return '\n'.join(xml)

    def generate_default_class(self) -> str:
        xml = []
        red_wall_parameters = 'type="box" zaxis="0 1 0" material="walls"'
        white_wall_parameters = 'type="box" zaxis="0 1 0" rgba="1 1 1 1"'
        length = (self.cell_size - self.wall_thickness) / 2
        height = self.wall_height / 2
        width = self.wall_thickness / 2

        xml.append('  <default>')
        xml.append('    <default class="post">')
        xml.append(f'      <geom {red_wall_parameters} size="{width:.6f} {height:.6f} {width:.6f}"/>')
        xml.append('    </default>')
        xml.append('    <default class="post_w">')
        xml.append(f'      <geom {white_wall_parameters} size="{width:.6f} {height:.6f} {width:.6f}"/>')
        xml.append('    </default>')
        xml.append('    <default class="hwall">')
        xml.append(f'      <geom {red_wall_parameters} size="{length:.6f} {height:.6f} {width:.6f}"/>')
        xml.append('    </default>')
        xml.append('    <default class="hwall_w">')
        xml.append(f'      <geom {white_wall_parameters} size="{length:.6f} {height:.6f} {width:.6f}"/>')
        xml.append('    </default>')
        xml.append('    <default class="vwall">')
        xml.append(f'      <geom {red_wall_parameters} size="{width:.6f} {height:.6f} {length:.6f}"/>')
        xml.append('    </default>')
        xml.append('    <default class="vwall_w">')
        xml.append(f'      <geom {white_wall_parameters} size="{width:.6f} {height:.6f} {length:.6f}"/>')
        xml.append('    </default>')
        xml.append('  </default>\n')
        return '\n'.join(xml)

    def generate_top_camera_view(self) -> str:
        xml = []
        maze_center = (self.maze_width * self.cell_size / 2, self.maze_height * self.cell_size / 2)
        xml.append(f'    <camera name="top_view" orthographic="true" fovy="3.5" pos="{maze_center[0]:.6f} {maze_center[1]:.6f} 4.5" xyaxes="1.0 0.0 0.0 0.0 1.0 0.0"/>')
        return '\n'.join(xml)

    def generate_xml(self, horizontal_walls: list[list[bool]], vertical_walls: list[list[bool]]) -> str:
        xml_lines = []
        xml_lines.append(self.add_header())
        xml_lines.append('<mujoco>')
        xml_lines.append(self.generate_assets())
        xml_lines.append(self.generate_default_class())
        xml_lines.append('  <worldbody>')
        xml_lines.append(self.generate_top_camera_view())
        xml_lines.extend(self.generate_posts_xml())
        xml_lines.extend(self.generate_horizontal_walls_xml(horizontal_walls))
        xml_lines.extend(self.generate_vertical_walls_xml(vertical_walls))
        xml_lines.append('  </worldbody>')
        xml_lines.append('</mujoco>')
        return '\n'.join(xml_lines)

def main():
    parser = argparse.ArgumentParser(description="Generate MuJoCo XML maze walls from text maze file")
    parser.add_argument("maze_file", help="Path to the maze text file")
    parser.add_argument("-o", "--output", help="Output XML file (default: maze.xml)")
    parser.add_argument("--cell-size", type=float, default=0.18, help="Cell size in meters (default: 0.18)")
    parser.add_argument("--wall-thickness", type=float, default=0.0126, help="Wall thickness in meters (default: 0.0126)")
    parser.add_argument("--wall-height", type=float, default=0.05, help="Wall height in meters (default: 0.05)")

    args = parser.parse_args()

    if not Path(args.maze_file).exists():
        print(f"Error: Maze file '{args.maze_file}' not found")
        sys.exit(1)

    output_file = args.output or "maze.xml"

    try:
        maze_parser = MazeParser(args.cell_size, args.wall_thickness)
        lines = maze_parser.parse_maze_file(args.maze_file)
        horizontal_walls = maze_parser.extract_horizontal_walls(lines)
        vertical_walls = maze_parser.extract_vertical_walls(lines)
        start, goals = maze_parser.find_start_and_goals(lines)
        print(f"Start point: {start}, Goals: {goals}")
        xml_content = maze_parser.generate_xml(horizontal_walls, vertical_walls)

        with open(output_file, 'w') as f:
            f.write(xml_content)

        print(f"Generated XML maze walls saved to '{output_file}'")

    except ValueError as e:
        print(f"Error parsing maze file: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

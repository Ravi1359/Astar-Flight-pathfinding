# Astar-Flight-pathfinding

This repository contains a simple pathfinding API that uses the A* algorithm to find the shortest path between two geographical coordinates. The distance calculation is based on the Haversine formula, which takes into account the Earth's curvature. The API is built using Flask.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [API Endpoint](#api-endpoint)
  - [POST /path](#post-path)
- [A* Algorithm](#a-algorithm)
- [Haversine Function](#haversine-function)
- [Neighbor Function](#neighbor-function)
- [Example](#example)

## Installation

1. Clone the repository:
    ```bash
    git clone <repository_url>
    cd <repository_directory>
    ```

2. Create a virtual environment and activate it:
    ```bash
    python3 -m venv venv
    source venv/bin/activate
    ```

3. Install the required packages:
    ```bash
    pip install Flask
    ```

## Usage

1. Start the Flask application:
    ```bash
    python app.py
    ```

2. The API will be available at `http://0.0.0.0:5000`.

## API Endpoint

### POST /path

Finds the shortest path between two geographical coordinates.

- **URL**: `/path`
- **Method**: `POST`
- **Content-Type**: `application/json`
- **Request Body**:
    ```json
    {
        "start": [latitude, longitude],
        "goal": [latitude, longitude]
    }
    ```
  - `start`: Starting coordinate as a list `[latitude, longitude]`.
  - `goal`: Goal coordinate as a list `[latitude, longitude]`.

- **Response**:
  - **Success** (`200 OK`):
    ```json
    {
        "path": [
            [latitude, longitude],
            [latitude, longitude],
            ...
        ]
    }
    ```
  - **Error** (`404 Not Found`):
    ```json
    {
        "error": "No path found"
    }
    ```

## A* Algorithm

The A* algorithm is implemented in the `astar` function. It uses the Haversine function to calculate the heuristic distance between coordinates and a priority queue (heap) to explore the most promising paths first.

### `astar` Function

```python
def astar(start, goal, neighbors_func):
    # Implementation details

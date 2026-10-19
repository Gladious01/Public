#include <iostream>
#include <vector>
#include <random>
#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

using namespace std;

struct Piece {
    std::vector<int> shape; // Shape of the piece (its array)
    int width;             // Width of the piece
};

const Piece pieces[] = {
    {
        {
            1,
            1
        }
        ,1
    },
    {
        {
            1,
            1,
            1
        }
        ,1
    },
    {
        {
            1,
            1,
            1,
            1
        }
        ,1
    },
    {
        {
            1, 1,
            1, 1
        }
        ,2
    },
    {
        {
            1, 1,
            1, 1,
            1, 1
        }
        ,2
    },
    {
        {
            1, 1, 1
        }
        ,3
    },
    {
        {
            1, 1, 1,
            1, 1, 1
        }
        ,3
    },
    {
        {
            1, 1, 1,
            1, 1, 1,
            1, 1, 1
        }
        ,3
    },
    {
        {
            1, 1, 1, 1
        }
        ,4
    }
};

int board[64] = { 0 };

Piece getRandomPiece() {
    static random_device rd;
    static mt19937 gen(rd());
    uniform_int_distribution<int> dist(0, sizeof(pieces) / sizeof(pieces[0]) - 1);

    return pieces[dist(gen)]; // Return a random piece
}

// Function to print a array into a grid
template <typename arr>
void printGrid(const arr& grid, int width, int size) {
    for (int i = 0; i < size; i++) {
        cout << (grid[i] == 1 ? "█" : "░") << "|";
        if ((i + 1) % width == 0) cout << endl;
    }
    cout << endl;
}

void checkPoints(int board[64], int index) {
    int row = index / 8;
    int col = index % 8;

    bool rowPerfect = true, colPerfect = true;

    // Check entire row
    for (int i = 0; i < 8; i++) {
        if (board[row * 8 + i] != 1) {
            rowPerfect = false;
            break;
        }
    }

    // Check entire column
    for (int i = 0; i < 8; i++) {
        if (board[i * 8 + col] != 1) {
            colPerfect = false;
            break;
        }
    }

    // If row is perfect, set all its elements to 0
    if (rowPerfect) {
        for (int i = 0; i < 8; i++) {
            board[row * 8 + i] = 0;
        }
    }

    // If column is perfect, set all its elements to 0
    if (colPerfect) {
        for (int i = 0; i < 8; i++) {
            board[i * 8 + col] = 0;
        }
    }
}

void placePiece(int pos, const vector<int>& piece, int width) {
    int height = piece.size() / width;

    // Check if starting position is within bounds
    if (pos < 0 || pos >= 64) {
        cout << "\nError: Position out of bounds\n";
        return;
    }

    int startCol = pos % 8;
    if (startCol + width > 8) {
        cout << "\nError: Piece doesn't fit in this row\n";
        return;
    }

    int startRow = pos / 8;
    if (startRow + height > 8) {
        cout << "\nError: Piece doesn't fit in this column\n";
        return;
    }

    // Check for collisions before placing the piece
    for (int i = 0; i < piece.size(); i++) {
        int rowOffset = i / width;
        int colOffset = i % width;

        int index = (startRow + rowOffset) * 8 + (startCol + colOffset);

        if (index >= 64) {
            cout << "\nError: Piece placement exceeds grid size\n";
            return;
        }

        if (board[index] != 0) {
            cout << "\nError: Space not valid\n";
            return;
        }
    }

    // Placing the piece
    for (int i = 0; i < piece.size(); i++) {
        int rowOffset = i / width;
        int colOffset = i % width;

        int index = (startRow + rowOffset) * 8 + (startCol + colOffset);
        board[index] = piece[i];

        checkPoints(board, index);
    }
}

int main() {
    #ifdef _WIN32
        SetConsoleOutputCP(CP_UTF8);  // Windows-specific code to set code page to UTF-8
    #endif
    while (true) {
        Piece piece = getRandomPiece();

        // Print board
        cout << "Board:\n";
        printGrid(board, 8, sizeof(board) / sizeof(board[0]));

        // Print piece
        cout << "Piece:\n";
        printGrid(piece.shape, piece.width, piece.shape.size());

        // Ask user for position to place piece
        cout << "Enter position to place piece (1-64) or type 'quit' to exit: ";
        string input;
        cin >> input;

        // Check for "quit" input
        if (input == "quit") {
            break;
        }

        // Convert input to integer
        int pos = -1;
        try {
            pos = stoi(input);
            pos -= 1;
        }
        catch (const exception& e) {
            cout << "Invalid input. Please enter a number between 1 and 64 or 'quit'.\n";
            continue;
        }

        // Place the piece
        placePiece(pos, piece.shape, piece.width);
    }

    cout << "Game over. Thank you for playing!\n";
    return 0;
}
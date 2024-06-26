#include "Interface.h"
#include "input.h"
#include "ansi.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <algorithm>

using namespace std;

bool Interface::init(){
    categoryMenu();
    if (graph_ == nullptr){
        cout << "Error reading the dataset files!\n";
        return false;
    }
    clearScreen();
    return true;
}

void Interface::clearScreen() {
#ifdef __unix__
    system("clear");
#else
    system("cls");
#endif
}

void Interface::printTop() {
    std::string s;
    for (int _ = 0; _ < 78; _++){
        s += "─";
    }
    std::cout << "┌" << s << "┐" << '\n';
    std::string title = "Travelling Salesperson Problem (G15_02)";
    std::cout << "│" << std::string(20, ' ') << CYAN << BOLD << title << RESET << std::string(19, ' ') << "│" << '\n';
}

void Interface::printBottom() {
    std::string s;
    for (int _ = 0; _ < 78; _++){
        s += "─";
    }
    std::cout << "└" << s << "┘" << '\n';
}

void Interface::waitInput() {
    initCapture();
    std::cout << HIDE_CURSOR;
    cout << '\n' << std::string(25, ' ') << FAINT << "< Press " << RESET << BOLD << "ENTER" << RESET << FAINT << " to continue >\n" << RESET;

    Press press;
    do {
        press = getNextPress();
    } while  (press != RET);
    endCapture();
}

int Interface::receiveVertexId() {
    int id;

    std::cout << FAINT << "Starting Vertex (0-" << graph_->getVertexSet().size()-1 << "): " << RESET;

    cin >> id;
    while (!cin || graph_->findVertex(id) == nullptr) {
        cout << BOLD << RED << "│ Invalid Vertex ID │ " << RESET;
        std::cout << FAINT << "Starting Vertex (0-" << graph_->getVertexSet().size()-1 << "): " << RESET;
        if (!cin) {
            cin.clear();
            cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        cin >> id;
    }
    cout << '\n';
    getNextPress(); // Skip enter

    return 0;
}

void Interface::printDatasetsOptions(const vector<std::string> &options, int choice){
    std::cout << "│" << std::string(4, ' ') << std::setw(74) << std::left << options[options.size()-1] << "│" << '\n';

    for (int idx = 1; idx < (int)options.size() - 1; idx++){
        int w = 73;
        if (idx >= 10) {
            w--;
        }
        if (choice == idx){
            std::cout << "│" << BOLD << GREEN << " [" << idx << "] " << RESET << BOLD << std::setw(w) << std::left << options[idx] << RESET << "│" << '\n';
        }
        else {
            std::cout << "│" << GREEN << " [" << idx << "] " << RESET << FAINT << std::setw(w) << std::left << options[idx] << RESET << "│" << '\n';
        }
    }
    if (choice == 0){
        std::cout << "│" << BOLD << RED << " [0] " << RESET << BOLD << std::setw(73) << std::left << options[0] << RESET "│" << '\n';
    }
    else {
        std::cout << "│" << RED << " [0] " << RESET << FAINT << std::setw(73) << std::left << options[0] << RESET << "│" << '\n';
    }
}

void Interface::printAlgorithmOptions(const vector<std::string> &options, int choice){
    std::cout << "│" << std::string(4, ' ') << std::setw(74) << std::left << options[options.size()-1] << "│" << '\n';

    for (int idx = 1; idx < 3; idx++){
        int space = 73;
        if (idx >= 10){
            space--;
        }
        if (choice == idx){
            std::cout << "│" << BOLD << GREEN << " [" << idx << "] " << RESET << BOLD << std::setw(space) << std::left << options[idx] << RESET << "│" << '\n';
        }
        else {
            std::cout << "│" << GREEN << " [" << idx << "] " << RESET << FAINT << std::setw(space) << std::left << options[idx] << RESET << "│" << '\n';
        }
    }
    for (int idx = 3; idx < (int)options.size() - 4; idx++){
        int space = 73;
        if (idx >= 10){
            space--;
        }
        if (choice == idx){
            std::cout << "│" << BOLD << YELLOW << " [" << idx << "] " << RESET << BOLD << std::setw(space) << std::left << options[idx] << RESET << "│" << '\n';
        }
        else {
            std::cout << "│" << YELLOW << " [" << idx << "] " << RESET << FAINT << std::setw(space) << std::left << options[idx] << RESET << "│" << '\n';
        }
    }

    int space = 73;

    if (choice == (int)options.size()-4){
        std::cout << "│" << BOLD << MAGENTA << " [" << options.size()-4 << "] " << RESET << BOLD << std::setw(space) << std::left << options[options.size()-4] << RESET "│" << '\n';
    }
    else {
        std::cout << "│" << MAGENTA << " [" << options.size()-4 << "] " << RESET << FAINT << std::setw(space) << std::left << options[options.size()-4] << RESET << "│" << '\n';
    }

    if (choice == (int)options.size()-3){
        std::cout << "│" << BOLD << CYAN << " [" << options.size()-3 << "] " << RESET << BOLD << std::setw(space) << std::left << options[options.size()-3] << RESET "│" << '\n';
    }
    else {
        std::cout << "│" << CYAN << " [" << options.size()-3 << "] " << RESET << FAINT << std::setw(space) << std::left << options[options.size()-3] << RESET << "│" << '\n';
    }

    if (choice == (int)options.size()-2){
        std::cout << "│" << BOLD << BLUE << " [" << options.size()-2 << "] " << RESET << BOLD << std::setw(space) << std::left << options[options.size()-2] << RESET "│" << '\n';
    }
    else {
        std::cout << "│" << BLUE << " [" << options.size()-2 << "] " << RESET << FAINT << std::setw(space) << std::left << options[options.size()-2] << RESET << "│" << '\n';
    }

    if (choice == 0){
        std::cout << "│" << BOLD << RED << " [0] " << RESET << BOLD << std::setw(73) << std::left << options[0] << RESET "│" << '\n';
    }
    else {
        std::cout << "│" << RED << " [0] " << RESET << FAINT << std::setw(73) << std::left << options[0] << RESET << "│" << '\n';
    }
}

void Interface::mainMenu() {
    initCapture();
    std::cout << HIDE_CURSOR;
    std::vector<std::string> options =
            {"Quit",
             "Backtracking Algorithm",
             "Held-Karp Algorithm",
             "Nearest Neighbour Heuristic",
             "Double Minimum Spanning Tree Heuristic",
             "Christofides* Heuristic",
             "Shortcutting Christofides* Heuristic",
             "Choose Best Algorithm",
             "Statistics",
             "Choose your operation:"
            };


    int choice = 1;
    Press press;
    do {
        clearScreen();
        printTop();
        printAlgorithmOptions(options, choice);
        printBottom();
        press = getNextPress();
        if (press == UP) {choice -= 1; choice += (options.size()-1);}
        else if (press == DOWN) {choice += 1;}
        choice = choice % (options.size()-1);
    } while (press != RET);
    endCapture();

    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    std::chrono::duration<double> execution{};
    double result = NAN;
    int startId;
    std::string title;
    switch (choice) {
        case 1: {
            start = chrono::high_resolution_clock::now();
            result = graph_->backtrackingTsp();
            end = chrono::high_resolution_clock::now();
            title = "Backtracking";
            execution = end - start;
            stats_.push_back({title, result, execution.count()});
            break;
        }
        case 2: {
            if (graph_->getVertexSet().size() > 64) {
                cout << BOLD << "Graph too big" << RESET << " for Held-Karp algorithm" << FAINT " (no of vertices > 64)" << RESET << '\n';
                cout << BOLD << "The algorithm will not be performed!" << RESET << '\n';
                waitInput();
                return;
            }
            start = chrono::high_resolution_clock::now();
            result = graph_->heldKarpTsp();
            end = chrono::high_resolution_clock::now();
            title = "Held-Karp";
            execution = end - start;
            stats_.push_back({title, result, execution.count()});
            break;
        }
        case 3: {
            startId = receiveVertexId();
            start = chrono::high_resolution_clock::now();
            result = graph_->nearestNeighbourTsp(startId);
            end = chrono::high_resolution_clock::now();
            title = "Nearest Neighbor";
            execution = end - start;
            stats_.push_back({title, result, execution.count()});
            break;
        }
        case 4: {
            startId = receiveVertexId();
            start = chrono::high_resolution_clock::now();
            result = graph_->doubleMstTsp(startId);
            end = chrono::high_resolution_clock::now();
            title = "Double MST";
            execution = end - start;
            stats_.push_back({title, result, execution.count()});
            break;
        }
        case 5: {
            startId = receiveVertexId();
            start = chrono::high_resolution_clock::now();
            result = graph_->christofidesStarTsp(startId);
            end = chrono::high_resolution_clock::now();
            title = "Christofides*";
            execution = end - start;
            stats_.push_back({title, result, execution.count()});
            break;
        }
        case 6: {
            startId = receiveVertexId();
            start = chrono::high_resolution_clock::now();
            result = graph_->shortcuttingChristofidesStarTsp(startId);
            end = chrono::high_resolution_clock::now();
            title = "Shortcutting Christofides*";
            execution = end - start;
            stats_.push_back({title, result, execution.count()});
            break;
        }
        case 7: {
            int numVertices, numEdges;

            numVertices = (int)graph_->getVertexSet().size();
            numEdges = graph_->getNumEdges();
            if (numVertices < 25) {
                cout << "Number of vertices: " << numVertices << FAINT << " (< 25)" << RESET << '\n';
                cout << BOLD << "  Choosing Held-Karp algorithm" << '\n' << '\n';
                start = chrono::high_resolution_clock::now();
                result = graph_->heldKarpTsp();
                end = chrono::high_resolution_clock::now();
            } else if (numEdges < (numVertices - 1) * numVertices / 2) {
                cout << "Number of vertices: " << numVertices << FAINT << " (>= 25)" << RESET << '\n';
                cout << "Graph is not fully connected" << '\n';
                cout << BOLD << "  Choosing Shortcutting Christofides* heuristic" << RESET << FAINT << " (starting in 0)" << RESET << '\n' << '\n';
                start = chrono::high_resolution_clock::now();
                result = graph_->shortcuttingChristofidesStarTsp(0);
                end = chrono::high_resolution_clock::now();
            } else if (numVertices < 1000) {
                cout << "Number of vertices: " << numVertices << FAINT << " (>= 25 and < 1000)" << RESET << '\n';
                cout << "Graph is fully connected" << '\n';
                cout << BOLD << "  Choosing Christofides* heuristic" << RESET << '\n' << '\n';
                start = chrono::high_resolution_clock::now();
                result = graph_->christofidesStarTsp(0);
                end = chrono::high_resolution_clock::now();
            } else {
                cout << "Number of vertices: " << numVertices << FAINT << " (> 1000)" << RESET << '\n';
                cout << "Graph is fully connected" << '\n';
                cout << BOLD << "  Choosing Nearest Neighbor heuristic" << RESET << '\n' << '\n';
                start = chrono::high_resolution_clock::now();
                result = graph_->nearestNeighbourTsp(0);
                end = chrono::high_resolution_clock::now();
            }

            execution = end - start;
            break;
        }
        case 8: {
            if (stats_.empty()){
                cout << "No statistics calculated to display\n";
            }
            else {
                statistics();
            }
            waitInput();
            return;
        }
        case 0:
            exitMenu();
            break;
    }

    if (result > 0 && !isinf(result) && !isnan(result))
        cout << BOLD << BLUE << "Result: " << RESET << fixed << setprecision(3) << result << FAINT << " m" << RESET << '\n';
    else
        cout << BOLD << BLUE << FAINT << "No results found" << RESET << '\n';
    cout << BOLD << BLUE << "Execution: " << RESET << fixed << setprecision(10) << execution.count() << FAINT << " s" << RESET << '\n';
    waitInput();
}

void Interface::categoryMenu() {
    initCapture();
    std::cout << HIDE_CURSOR;
    std::vector<std::string> options =
            {"Quit",
             "Extra Fully Connected Graphs",
             "Real World Graphs",
             "Toy Graphs",
             "Custom dataset (see README)",
             "Choose the Dataset"
            };

    int choice = 1;
    Press press;
    do {
        clearScreen();
        printTop();
        printDatasetsOptions(options, choice);
        printBottom();
        press = getNextPress();
        if (press == UP) {choice -= 1; choice += (options.size()-1);}
        else if (press == DOWN) {choice += 1;}
        choice = choice % (options.size()-1);
    } while (press != RET);

    endCapture();

    switch (choice) {
        case 1: extraFullyConnectedMenu(); break;
        case 2: realWorldMenu(); break;
        case 3: toyMenu(); break;
        case 4:
            graph_ = Graph::parseRealWorldGraph("../graphs/Custom/nodes.csv", "../graphs/Custom/edges.csv");
            break;
        default:
            exitMenu();
    }
}

void Interface::extraFullyConnectedMenu() {
    initCapture();
    std::cout << HIDE_CURSOR;
    std::vector<std::string> options =
            {"Back",
             "Extra Fully Connected: 25 edges",
             "Extra Fully Connected: 50 edges",
             "Extra Fully Connected: 75 edges",
             "Extra Fully Connected: 100 edges",
             "Extra Fully Connected: 200 edges",
             "Extra Fully Connected: 300 edges",
             "Extra Fully Connected: 400 edges",
             "Extra Fully Connected: 500 edges",
             "Extra Fully Connected: 600 edges",
             "Extra Fully Connected: 700 edges",
             "Extra Fully Connected: 800 edges",
             "Extra Fully Connected: 900 edges",
             "Choose the Dataset"
            };

    int choice = 1;
    Press press;
    do {
        clearScreen();
        printTop();
        printDatasetsOptions(options, choice);
        printBottom();
        press = getNextPress();
        if (press == UP) {choice -= 1; choice += (options.size()-1);}
        else if (press == DOWN) {choice += 1;}
        choice = choice % (options.size()-1);
    } while (press != RET);

    endCapture();

    string nodeFilename = "../graphs/Extra Fully Connected/nodes.csv", edgeFilename;
    switch (choice) {
        case 1: edgeFilename = "../graphs/Extra Fully Connected/edges_25.csv"; break;
        case 2: edgeFilename = "../graphs/Extra Fully Connected/edges_50.csv"; break;
        case 3: edgeFilename = "../graphs/Extra Fully Connected/edges_75.csv"; break;
        case 4: edgeFilename = "../graphs/Extra Fully Connected/edges_100.csv"; break;
        case 5: edgeFilename = "../graphs/Extra Fully Connected/edges_200.csv"; break;
        case 6: edgeFilename = "../graphs/Extra Fully Connected/edges_300.csv"; break;
        case 7: edgeFilename = "../graphs/Extra Fully Connected/edges_400.csv"; break;
        case 8: edgeFilename = "../graphs/Extra Fully Connected/edges_500.csv"; break;
        case 9: edgeFilename = "../graphs/Extra Fully Connected/edges_600.csv"; break;
        case 10: edgeFilename = "../graphs/Extra Fully Connected/edges_700.csv"; break;
        case 11: edgeFilename = "../graphs/Extra Fully Connected/edges_800.csv"; break;
        case 12: edgeFilename = "../graphs/Extra Fully Connected/edges_900.csv"; break;
        default:
            categoryMenu();
    }
    graph_ = Graph::parseMediumGraph(nodeFilename, edgeFilename);
}

void Interface::realWorldMenu() {
    initCapture();
    std::cout << HIDE_CURSOR;
    std::vector<std::string> options =
            {"Back",
             "Real World Graphs: 1",
             "Real World Graphs: 2",
             "Real World Graphs: 3",
             "Choose the Dataset"
            };

    int choice = 1;
    Press press;
    do {
        clearScreen();
        printTop();
        printDatasetsOptions(options, choice);
        printBottom();
        press = getNextPress();
        if (press == UP) {choice -= 1; choice += (options.size()-1);}
        else if (press == DOWN) {choice += 1;}
        choice = choice % (options.size()-1);
    } while (press != RET);

    endCapture();

    switch (choice) {
        case 1: graph_ = Graph::parseRealWorldGraph("../graphs/Real World/graph1/nodes.csv", "../graphs/Real World/graph1/edges.csv"); break;
        case 2: graph_ = Graph::parseRealWorldGraph("../graphs/Real World/graph2/nodes.csv", "../graphs/Real World/graph2/edges.csv"); break;
        case 3: graph_ = Graph::parseRealWorldGraph("../graphs/Real World/graph3/nodes.csv", "../graphs/Real World/graph3/edges.csv"); break;
        default:
            categoryMenu();
    }
}

void Interface::toyMenu() {
    initCapture();
    std::cout << HIDE_CURSOR;
    std::vector<std::string> options =
            {"Back",
             "Toy Graphs: Shipping",
             "Toy Graphs: Stadiums",
             "Toy Graphs: Tourism",
             "Choose the Dataset"
            };

    int choice = 1;
    Press press;
    do {
        clearScreen();
        printTop();
        printDatasetsOptions(options, choice);
        printBottom();
        press = getNextPress();
        if (press == UP) {choice -= 1; choice += (options.size()-1);}
        else if (press == DOWN) {choice += 1;}
        choice = choice % (options.size()-1);
    } while (press != RET);

    endCapture();

    switch (choice) {
        case 1: graph_ = Graph::parseToyGraph("../graphs/Toy/shipping.csv"); break;
        case 2: graph_ = Graph::parseToyGraph("../graphs/Toy/stadiums.csv"); break;
        case 3: graph_ = Graph::parseToyGraph("../graphs/Toy/tourism.csv"); break;
        default:
            categoryMenu();
    }
}

void Interface::statistics() {
    std::sort(stats_.begin(), stats_.end(),
              [](const Statistic& s1, const Statistic& s2){return s1.result < s2.result || (s1.result == s2.result && s1.time < s2.time);});
    cout << BOLD << INVERT << std::string(15,' ') << "Algorithm" << std::string(16,' ') << left << setw(20) << "TSP Result" << setw(20) << "Time" << RESET << '\n';
    for (const Statistic& s : stats_){
        if (s.result < 0 || isinf(s.result))
            continue;
        cout << "│" << std::string(2, ' ') << BOLD << left << setw(37) << s.algorithm << RESET << fixed << setprecision(3) << setw(20) << s.result << setprecision(10) << setw(19) << s.time << "│" << '\n';
    }
    printBottom();
}

void Interface::exitMenu() {
    std::cout << "Closing the app...\n";
    exit(0);
}

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
    if (graph == nullptr){
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
    for (int idx = 3; idx < (int)options.size() - 3; idx++){
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

    if (choice == (int)options.size()-3){
        std::cout << "│" << BOLD << MAGENTA << " [" << options.size()-3 << "] " << RESET << BOLD << std::setw(space) << std::left << options[options.size()-3] << RESET "│" << '\n';
    }
    else {
        std::cout << "│" << MAGENTA << " [" << options.size()-3 << "] " << RESET << FAINT << std::setw(space) << std::left << options[options.size()-3] << RESET << "│" << '\n';
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
             "Double Minimum Spanning Tree Heuristic",
             "Nearest Neighbour Heuristic",
             "Christofides* Heuristic",
             "Real World Heuristic",
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
    std::string title;
    switch (choice) {
        case 1: {
            start = chrono::high_resolution_clock::now();
            result = graph->backtrackingTsp(0);
            end = chrono::high_resolution_clock::now();
            title = "Backtracking";
            execution = end - start;
            stats.push_back({title, result, execution.count()});
            break;
        }
        case 2: {
            start = chrono::high_resolution_clock::now();
            result = graph->heldKarpTsp(0);
            end = chrono::high_resolution_clock::now();
            title = "Held-Karp";
            execution = end - start;
            stats.push_back({title, result, execution.count()});
            break;
        }
        case 3: {
            start = chrono::high_resolution_clock::now();
            result = graph->doubleMstTsp(0);
            end = chrono::high_resolution_clock::now();
            title = "Double MST";
            execution = end - start;
            stats.push_back({title, result, execution.count()});
            break;
        }
        case 4: {
            start = chrono::high_resolution_clock::now();
            result = graph->nearestNeighbourTsp(0);
            end = chrono::high_resolution_clock::now();
            title = "Nearest Neighbor";
            execution = end - start;
            stats.push_back({title, result, execution.count()});
            break;
        }
        case 5: {
            start = chrono::high_resolution_clock::now();
            result = graph->christofidesTsp(0);
            end = chrono::high_resolution_clock::now();
            title = "Christofides*";
            execution = end - start;
            stats.push_back({title, result, execution.count()});
            break;
        }
        case 6: {
            start = chrono::high_resolution_clock::now();
            result = graph->realWorldTsp(0);
            end = chrono::high_resolution_clock::now();
            title = "Real World";
            execution = end - start;
            stats.push_back({title, result, execution.count()});
            break;
        }
        case 7: {
            statistics();
            waitInput();
            return;
        }
        case 0:
            exitMenu();
            break;
    }

    cout << BOLD << MAGENTA << string(15, ' ') << title << RESET << '\n';
    cout << BOLD << BLUE << "Result: " << RESET << fixed << setprecision(3) << result << FAINT << " m" << RESET << '\n';
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
            graph = Graph::parseRealWorldGraph("../graphs/Custom/nodes.csv", "../graphs/Custom/edges.csv");
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
    graph = Graph::parseMediumGraph(nodeFilename, edgeFilename);
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
        case 1: graph = Graph::parseRealWorldGraph("../graphs/Real World/graph1/nodes.csv", "../graphs/Real World/graph1/edges.csv"); break;
        case 2: graph = Graph::parseRealWorldGraph("../graphs/Real World/graph2/nodes.csv", "../graphs/Real World/graph2/edges.csv"); break;
        case 3: graph = Graph::parseRealWorldGraph("../graphs/Real World/graph3/nodes.csv", "../graphs/Real World/graph3/edges.csv"); break;
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
        case 1: graph = Graph::parseToyGraph("../graphs/Toy/shipping.csv"); break;
        case 2: graph = Graph::parseToyGraph("../graphs/Toy/stadiums.csv"); break;
        case 3: graph = Graph::parseToyGraph("../graphs/Toy/tourism.csv"); break;
        default:
            categoryMenu();
    }
}

void Interface::statistics() {
    std::sort(stats.begin(), stats.end(),
              [](const Statistic& s1, const Statistic& s2){return s1.result < s2.result || (s1.result == s2.result && s1.time < s2.time);});
    cout << BOLD << INVERT << std::string(15,' ') << "Algorithm" << std::string(16,' ') << left << setw(20) << "TSP Result" << setw(20) << "Time" << RESET << '\n';
    for (const Statistic& s : stats){
        cout << "│" << std::string(2, ' ') << BOLD << left << setw(37) << s.algorithm << RESET << fixed << setprecision(3) << setw(20) << s.result << setprecision(10) << setw(19) << s.time << "│" << '\n';
    }
    printBottom();
}

void Interface::exitMenu() {
    std::cout << "Closing the app...\n";
    exit(0);
}
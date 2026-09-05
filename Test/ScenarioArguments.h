#pragma once

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

namespace ModelDevelop::TestSupport {
    struct NumericOption {
        const char *name;
        double *value;
        double minimum;
        double maximum;
        const char *unit;
        const char *description;
    };

    struct ChoiceOption {
        const char *name;
        std::string *value;
        std::vector<std::string> choices;
        const char *description;
    };

    enum class ParseResult {
        Ok,
        Help,
        Error
    };

    inline void printUsage(
        const char *program,
        const std::vector<NumericOption> &numericOptions,
        const std::vector<double> &numericDefaults,
        const std::vector<ChoiceOption> &choiceOptions,
        const std::vector<std::string> &choiceDefaults) {
        std::cout << "Usage: " << program << " [options]\n"
                  << "Options:\n"
                  << "  --help\n"
                  << "      Show this help and exit.\n";

        for (std::size_t index = 0; index < numericOptions.size(); ++index) {
            const auto &option = numericOptions[index];
            std::cout << "  " << option.name << " <number>\n"
                      << "      " << option.description
                      << " (range [" << option.minimum << ", " << option.maximum << "] "
                      << option.unit << ", default " << numericDefaults[index] << ")\n";
        }

        for (std::size_t index = 0; index < choiceOptions.size(); ++index) {
            const auto &option = choiceOptions[index];
            std::cout << "  " << option.name << " <choice>\n"
                      << "      " << option.description << " (allowed: ";
            for (std::size_t choiceIndex = 0; choiceIndex < option.choices.size(); ++choiceIndex) {
                if (choiceIndex > 0) {
                    std::cout << " | ";
                }
                std::cout << option.choices[choiceIndex];
            }
            std::cout << ", default " << choiceDefaults[index] << ")\n";
        }
    }

    inline bool parseFiniteDouble(const char *text, double &value) {
        if (text == nullptr || *text == '\0') {
            return false;
        }

        char *end = nullptr;
        errno = 0;
        const double parsed = std::strtod(text, &end);
        if (errno == ERANGE || end == text || *end != '\0' || !std::isfinite(parsed)) {
            return false;
        }

        value = parsed;
        return true;
    }

    inline ParseResult parseArguments(
        const int argc,
        char *argv[],
        const std::vector<NumericOption> &numericOptions,
        const std::vector<ChoiceOption> &choiceOptions) {
        std::vector<double> numericDefaults;
        numericDefaults.reserve(numericOptions.size());
        for (const auto &option : numericOptions) {
            numericDefaults.push_back(*option.value);
        }

        std::vector<std::string> choiceDefaults;
        choiceDefaults.reserve(choiceOptions.size());
        for (const auto &option : choiceOptions) {
            choiceDefaults.push_back(*option.value);
        }

        std::unordered_set<std::string> seen;
        for (int index = 1; index < argc; ++index) {
            const std::string argument = argv[index];
            if (argument == "--help") {
                printUsage(argv[0], numericOptions, numericDefaults, choiceOptions, choiceDefaults);
                return ParseResult::Help;
            }

            const NumericOption *matchedNumeric = nullptr;
            for (const auto &option : numericOptions) {
                if (argument == option.name) {
                    matchedNumeric = &option;
                    break;
                }
            }

            const ChoiceOption *matchedChoice = nullptr;
            if (matchedNumeric == nullptr) {
                for (const auto &option : choiceOptions) {
                    if (argument == option.name) {
                        matchedChoice = &option;
                        break;
                    }
                }
            }

            if (matchedNumeric == nullptr && matchedChoice == nullptr) {
                std::cerr << "Unknown option: " << argument << "\n"
                          << "Use --help to list supported options." << std::endl;
                return ParseResult::Error;
            }
            if (!seen.insert(argument).second) {
                std::cerr << "Duplicate option: " << argument << std::endl;
                return ParseResult::Error;
            }
            if (index + 1 >= argc || std::string(argv[index + 1]).rfind("--", 0) == 0) {
                std::cerr << "Missing value for option: " << argument << std::endl;
                return ParseResult::Error;
            }

            const char *valueText = argv[++index];
            if (matchedChoice != nullptr) {
                const std::string parsed = valueText;
                if (std::find(matchedChoice->choices.begin(), matchedChoice->choices.end(), parsed) == matchedChoice->choices.end()) {
                    std::cerr << "Option " << argument << " must be one of: ";
                    for (std::size_t choiceIndex = 0; choiceIndex < matchedChoice->choices.size(); ++choiceIndex) {
                        if (choiceIndex > 0) {
                            std::cerr << ", ";
                        }
                        std::cerr << matchedChoice->choices[choiceIndex];
                    }
                    std::cerr << "; got: " << parsed << std::endl;
                    return ParseResult::Error;
                }
                *matchedChoice->value = parsed;
                continue;
            }

            double parsed = 0.0;
            if (!parseFiniteDouble(valueText, parsed)) {
                std::cerr << "Option " << argument << " requires a finite number, got: "
                          << valueText << std::endl;
                return ParseResult::Error;
            }
            if (parsed < matchedNumeric->minimum || parsed > matchedNumeric->maximum) {
                std::cerr << "Option " << argument << " must be in ["
                          << matchedNumeric->minimum << ", " << matchedNumeric->maximum << "] "
                          << matchedNumeric->unit << ", got: " << parsed << std::endl;
                return ParseResult::Error;
            }

            *matchedNumeric->value = parsed;
        }

        return ParseResult::Ok;
    }

    inline ParseResult parseNumericArguments(
        const int argc,
        char *argv[],
        const std::vector<NumericOption> &options) {
        return parseArguments(argc, argv, options, {});
    }
}

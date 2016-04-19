/*
 * Tokenizer.h
 *
 * Class that separates a string in tokens, given a separator character.
 *
 *  Created on: Jul 10, 2014
 *      Author: Vítor E. Silva Souza (vitorsouza@gmail.com)
 */

#ifndef TOKENIZER_H_
#define TOKENIZER_H_

#include <sstream>
#include <string>
#include <vector>

namespace br_ufes_inf_nemo_cpp_util {

using namespace std;

class Tokenizer {
	/* A stream with the contents of the string that is to be separated. */
	stringstream stream;

	/* The separator character. */
	char separator;

public:
	Tokenizer(const string& str, char separator);
	virtual ~Tokenizer() { }

	/* Indicates if there is another token to read. */
	bool hasNext();

	/* Returns the next token. */
	const string next();

	/* Returns all the remaining tokens as a vector of strings. */
	vector<string> remaining();
};

}
#endif

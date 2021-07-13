package main

import (
	"bufio"
	"fmt"
	"log"
	"os"
	"regexp"
)

func main() {
	file, err := os.Open(os.Args[1])
	if err != nil {
		log.Fatal(err)
	}
	defer file.Close()

	seenHeader := false

	scanner := bufio.NewScanner(file)
	rh, err := regexp.Compile("Channel symbols")
	if err != nil {
		log.Fatal(err)
	}
	rt, err := regexp.Compile("^[ \t\n]*$") // empty line
	if err != nil {
		log.Fatal(err)
	}

	output := ""

	for scanner.Scan() {
		if rh.MatchString(scanner.Text()) {
			seenHeader = true
		} else if seenHeader == true {
			if seenHeader == true && rt.MatchString(scanner.Text()) {
				break
			} else {
				output = output + scanner.Text()
			}
		}
	}
	fmt.Print("const uint8_t FST4Wsymbols[FST4W_SYMBOL_COUNT] = { ")
	fmt.Printf("%c", output[0])
	for _, char := range output[1:] {
		fmt.Printf(", %c ", char)
	}
	fmt.Println("};")
	if err := scanner.Err(); err != nil {
		log.Fatal(err)
	}
}

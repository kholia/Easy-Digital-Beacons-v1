# https://raw.githubusercontent.com/emscripten-core/emscripten/master/src/shell_minimal.html
# https://gist.github.com/bd1es/a782e2529b8289288fadd35e407f6440

stuff:
	emcc hello.c wspr_enc_test.c wspr_enc.c nhash.c -s ALLOW_MEMORY_GROWTH=1 -s WASM=1 -s ERROR_ON_UNDEFINED_SYMBOLS=0 -s "EXPORTED_RUNTIME_METHODS=['ccall','cwrap']" -s EXPORTED_FUNCTIONS="['_malloc', '_main']" -o wspr_encoder.html --shell-file html_template/shell_minimal.html
	cp wspr_encoder* ~/repos/kholia.github.io/

copy:
	cp wspr_encoder* ~/repos/kholia.github.io/

run:
	python3 -m http.server

check:
	tidy wspr_encoder.html

format:
	astyle --options="formatter.conf" wspr_enc_test.c

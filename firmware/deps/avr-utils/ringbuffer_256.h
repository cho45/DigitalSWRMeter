typedef struct {
	uint8_t write_index;
	uint8_t read_index;
	uint8_t size;
	uint8_t data[255];
} ringbuffer;

int ringbuffer_get(ringbuffer* buffer) {
	if (buffer->size == 0) return 0;
	uint8_t ret;
	ret = buffer->data[buffer->read_index];
	// char の overflow にまかせる
	buffer->read_index++;
	buffer->size--;
	return ret;
}

void ringbuffer_put(ringbuffer* buffer, uint8_t data) {
	buffer->data[buffer->write_index] = data;
	buffer->write_index++;
	if (buffer->size < 255) {
		buffer->size++;
	} else {
		// overflowed
		buffer->read_index++;
	}
}

void ringbuffer_init(ringbuffer* buffer) {
	buffer->write_index = 0;
	buffer->read_index = 0;
	buffer->size = 0;
}



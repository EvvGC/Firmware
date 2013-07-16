/** \file printf.c
 * \brief Simplified printf() implementation
 * \details Simplified printf() implementation - prints formatted string to
 * stdout_ (USART). Only %s, %d, %x, %c and %% specifiers are supported
 * \author Freddie Chopin
 * \date 2009-10-02
 */

/******************************************************************************
* project: 750-750
* chip: STM32F103VB
* compiler: arm-none-eabi-gcc 4.4.1
*
* prefix: (none)
*
* available global functions:
* 	int printf_(const char *format, ...)
* 	int sprintf_(char *buffer, const char *format, ...)
*
* available local functions:
* 	int __fputc_(int character, printf_file_t *stream)
* 	int __vfprintf_(printf_file_t *stream, const char *format, va_list arg)
*
* available interrupt handlers:
******************************************************************************/

/*
+=============================================================================+
| includes
+=============================================================================+
*/

#include <stdarg.h>
#include <stdlib.h>

#include "printf.h"
//#include "usart.h"
#include "itoa.h"

/*
+=============================================================================+
| module variables
+=============================================================================+
*/

#if PRINTF_HAVE_PRINTF == 1

printf_file_t stdout_file = {
#if PRINTF_HAVE_PRINTF_SPRINTF == 1
		NULL,
#endif
		&usart_put_char, 0};	///< stdout file handle - usart_put_char() by default

#endif
/*
+=============================================================================+
| local functions' declarations
+=============================================================================+
*/

static int __fputc_(int character, printf_file_t *stream);
static int __vfprintf_(printf_file_t *stream, const char *format, va_list arg);

/*
+=============================================================================+
| global functions
+=============================================================================+
*/

#if PRINTF_HAVE_PRINTF == 1

/*------------------------------------------------------------------------*//**
* \brief Simplified printtf() - prints formatted string
* \details Prints a string to stdout_ (USART). Only %s, %d, %x, %c and %%
* specifiers are supported
*
* \param [in] format is a standard format string, additional parameters
* expected - one for each specifier
* \return number of written characters
*//*-------------------------------------------------------------------------*/

int printf_(const char *format, ...)
{
	va_list arg;
	int count;

	va_start(arg, format);
	count = __vfprintf_(stdout_, format, arg);
	va_end(arg);

	return count;
}

#endif

#if PRINTF_HAVE_SPRINTF == 1

/*------------------------------------------------------------------------*//**
* \brief Simplified sprinttf() - prints formatted string to buffer
* \details Prints a string to buffer. Only %s, %d, %x, %c and %% specifiers are
* supported
*
* \param [out] buffer is a buffer for string
* \param [in] format is a standard format string, additional parameters
* expected - one for each specifier
* \return number of written characters
*//*-------------------------------------------------------------------------*/

int sprintf_(char *buffer, const char *format, ...)
{
	printf_file_t stream;
	va_list arg;
	int count;

	stream.buffer = buffer;

#if PRINTF_HAVE_PRINTF == 1
	stream.put = NULL;
#endif

	va_start(arg, format);
	count = __vfprintf_(&stream, format, arg);
	va_end(arg);

	buffer[count]='\0';

	return count;
}

#endif

/*
+=============================================================================+
| local functions
+=============================================================================+
*/

/*------------------------------------------------------------------------*//**
* \brief Simplified fputc() - adds a character to file-stream
* \details Add one character to file-stream. Use (*put)() function pointer or
* *buffer - whichever is present. Increments the length of stream
*
* \param [in] character is a character which will be appended to stream
* \param [in, out] stream is a pointer to printf_file_t stream
* \return character which was passed as parameter
*//*-------------------------------------------------------------------------*/

static int __fputc_(int character, printf_file_t *stream)
{
#if PRINTF_HAVE_PRINTF_SPRINTF == 1
	if (stream->buffer != NULL)					// is buffer pointer valid?
#endif

#if PRINTF_HAVE_SPRINTF == 1
		*stream->buffer++ = (char)character;	// yes - just add the character to it
#endif

#if PRINTF_HAVE_PRINTF_SPRINTF == 1
	else if (stream->put != NULL)				// is put() function pointer valid?
#endif

#if PRINTF_HAVE_PRINTF == 1
		(*stream->put)((char)character);		// yes - use it to add one character to stream
#endif

	stream->length++;							// increment the length of stream

	return character;
}

/*------------------------------------------------------------------------*//**
* \brief Simplified vfprintf() - prints formatted string to stream
* \details Prints a string to stream. Only %s, %d, %x, %c and %%
* specifiers are supported.
*
* \param [in, out] stream is the file-stream which will be used as output
* \param [in] format is a standard format string, additional parameters
* expected - one for each specifier
* \param [in] arg is variable arguments list handle
* \return number of written characters
*//*-------------------------------------------------------------------------*/

static int __vfprintf_(printf_file_t *stream, const char *format, va_list arg)
{
	char character;

	stream->length=0;						// clear the current length of stream

	while ((character = *format++) != '\0')	// loop until termination character '\0'
	{
		if (character != '%')				// specifier found?
			__fputc_(character, stream);	// no - just print the character
		else								// yes
		{
			character=*format++;			// get the character after the specifier

			if (character == '%' || character == 'c')	// %% - print '%' or %c - print single char
			{
				if (character == 'c')		// was that %c?
					character = va_arg(arg, int);	// get the char from va_list
				__fputc_(character, stream);
				continue;
			}

			// %s, %d and %x - these require a string to be copied to stream
			if (character == 's' || character == 'd' || character == 'x')
			{
				char buffer[11];
				char* buffer_ptr;

				if (character == 's')		// %s - get the pointer to string
					buffer_ptr=va_arg(arg, char*);
				else						// %d or %x - convert the number to string
				{
					int base = (character == 'd' ? 10 : 16);

					buffer_ptr = itoa(va_arg(arg, int), buffer, base);
				}

				while ((character = *buffer_ptr++))	// copy the string to stream
					__fputc_(character, stream);
				continue;
			}
		}
	}

	return stream->length;
}

/*
+=============================================================================+
| ISRs
+=============================================================================+
*/

/******************************************************************************
* END OF FILE
******************************************************************************/

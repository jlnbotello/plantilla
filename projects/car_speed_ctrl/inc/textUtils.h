/** @addtogroup textUtils Utilidades de texto
   \brief Funciones para enviar texto y números por USART2
*  @{
*/
#ifndef INC_TEXTUTILS_H_
#define INC_TEXTUTILS_H_
#include "stdint.h"

/**\brief Convierte un valor a un representación ASCII
 *
 * Función para utilizar en combinación con \ref print(char *txt)
 *
 * \param[in] valor Valor a convertir
 * \param[out] toASCII Arreglo de caracteres que respresentan valor. Agrega '\0' al final
 */
void uint32ToASCII(uint32_t valor, char * toASCII);

/**\brief Determina los digitos de un número
 * \param[in] valor Número
 * \param[out] digitos Arreglo de digitos
 * \return Cantidad de dígitos
 */
uint8_t digitosDec(uint32_t valor, uint8_t * digitos);

#endif /* INC_TEXTUTILS_H_ */

/** @}*/

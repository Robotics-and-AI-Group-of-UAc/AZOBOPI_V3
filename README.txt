Esta versão é criada em Dezembro de 2023 para atender às dificuldades relativamente ao comportamento do robô.

A aposta nesta proposta é:
-- No que respeita ao movimento para a frente, garantir que o PID atua ao longo da trajetória do robô. Por exemplo, se se desloca
3 vezes para a frente, ele atua em toda a trajetória. Se vira duas vezes, ele atua em toda a trajetória.

Foram feitas as seguintes alterações:
-- Na viragem, assume-se diferença entre motores, a variável shift que tem de ser ajustada no arranque do robot. Na primeira vez em que se arranca o software, na interface que teremos que criar, é pedido ao professor para fazer alguns testes. Esses testes irão ser feitos de forma
a ter um valor entre os encoders após executar código em que se envia para ambos os motores valores de potencia iguais. 
Essa leitura feita 3 / 4 vezes irá dar um valor de shift a implementar nos motores. Esse programa deverá introduzir essas variações num ficheiro de texto ou noutro formato que é gerado por essa interface. Depois, ao descarregar o código para o robo, esse ficheiro é tido em conta, ajustando assim os valores de shift para o robo.

-- Em frente: Vou alterar o código de forma a que o robo registe ao longo do percurso que faz os desvios no enconder permitindo um
ajuste dos valores com mais tempo. Assim, se o utilizador carrega duas vezes o botao em frente, ele faz os calculos para a distancia maxima
deixando o PID atuar durante esse percurso - tenho de verificar se é possível.
-- Outra hipótese: Quando o robo se desvia para a direita ou esquerda, posso alterar o valor de shift. Tenho de recordar como é que faço para
ajustar a direita e a esquerda....

Allterações em:
08 Dezembro 2023





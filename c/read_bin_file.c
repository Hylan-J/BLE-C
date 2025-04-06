#include <stdio.h>
#include <stdlib.h>

// 修改后的函数，返回 data 的指针，并通过参数传递元素数量
double* read_refIQsamples_from_bin_file(const char *filename, size_t *num_refIQsamples)
{
    FILE *file = fopen(filename, "rb");
    if (file == NULL)
    {
        perror("Failed to open file.");
        return NULL;
    }

    // Get the size of the file
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    // Calculate the number of elements
    *num_refIQsamples = file_size / sizeof(double);

    // Allocate memory for the data
    double *refIQsamples = (double *)malloc(*num_refIQsamples * sizeof(double));
    if (refIQsamples == NULL)
    {
        perror("Failed to allocate memory.");
        fclose(file);
        return NULL;
    }

    // Read the data from the file
    size_t num_refIQsamples_read = fread(refIQsamples, sizeof(double), *num_refIQsamples, file);

    if (ferror(file))
    {
        perror("Failed to read data from file.");
        free(refIQsamples);
        fclose(file);
        return NULL;
    }

    fclose(file);

    printf("Data read from file:\n");
    for (size_t i = 0; i < num_refIQsamples_read; i++)
    {
        printf("%.15f ", refIQsamples[i]);
    }
    printf("\n");

    return refIQsamples;
}

// 使用示例
int main()
{
    const char *filename = "LE1M.bin";
    size_t num_refIQsamples;
    double *data = read_refIQsamples_from_bin_file(filename, &num_refIQsamples);

    if (data != NULL)
    {
        // 在这里可以使用 data 和 element_count
        printf("Successfully read %zu elements.\n", num_refIQsamples);

        // 使用完后释放内存
        free(data);
    }

    return 0;
}
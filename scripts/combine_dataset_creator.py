import cv2


def combine_image(start_num, read_dir_path, write_dir_path):
    images = []
    for i in range(start_num, start_num - 4, -1):
        images.append(cv2.imread(f"{read_dir_path}/image_{i}.png"))

    img01 = cv2.hconcat([images[0], images[1]])
    img23 = cv2.hconcat([images[2], images[3]])
    img = cv2.vconcat([img01, img23])

    cv2.imwrite(f"{write_dir_path}/image_{start_num}.png", img)


def main():
    date = "0606"
    type_name = "ref"  # ref or query
    start_num = 3
    end_num = 196

    read_dir_path = "/home/user/dataset/dataset_" + date + "_" + type_name
    write_dir_path = (
        "/home/user/dataset/dataset_" + date + "_" + type_name + "_combined"
    )

    for i in range(start_num, end_num + 1):
        combine_image(i, read_dir_path, write_dir_path)


if __name__ == "__main__":
    main()
